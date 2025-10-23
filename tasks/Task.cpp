/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Timeout.hpp>
#include <iodrivers_base/ConfigureGuard.hpp>
#include <usbl_seatrac/Protocol.hpp>

using namespace usbl_seatrac;
using namespace base;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    _safe_operational_pressure.set(samples::Pressure::fromBar(base::Time::now(), 1.01));
    setRuntimeErrorIOProcessingEnabled(true);
}

Task::~Task()
{
}

static Eigen::Quaterniond convertToOrientationQuaterniond(Status const& data)
{
    Eigen::Quaterniond orientation =
        Eigen::Quaterniond(Eigen::AngleAxisd(data.attitude.yaw / 10. / 180.0 * M_PI,
                               Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(data.attitude.pitch / 10. / 180.0 * M_PI,
                               Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(data.attitude.roll / 10. / 180.0 * M_PI,
                               Eigen::Vector3d::UnitX()));

    return orientation;
}

static samples::RigidBodyState convertToPositionRBS(base::Time const& time,
    protocol::AcousticFixPosition const& fix)
{
    samples::RigidBodyState rbs;
    rbs.time = time;
    rbs.position = Eigen::Vector3d(fix.position.north / 10.0,
        fix.position.east / 10.0,
        fix.position.depth / 10.0);
    return rbs;
}

void Task::configureUSBLSettings(
    usbl_seatrac::protocol::BeaconIdentificationCode xcvr_beacon_id,
    usbl_seatrac::protocol::TxMessageControl xcvr_tx_msgctrl,
    usbl_seatrac::protocol::StatusMode status_mode,
    base::Time xcvr_resp_time,
    base::Time xcvr_posflt_tmo,
    bool auto_vos,
    bool auto_pressure_ofs,
    bool auto_cal_mag,
    bool usbl_use_ahrs,
    bool xcvr_posflt_enable,
    bool xcvr_usbl_msgs,
    bool xcvr_fix_msgs,
    bool xcvr_diag_msgs,
    float xcvr_range_tmo)
{
    // Set initial settings
    protocol::Settings new_settings = mDriver->readSettings();

    uint8_t status_flags = status_mode;
    uint8_t enviromental_flags = auto_vos | (auto_pressure_ofs << 1);
    uint8_t ahrs_flags = auto_cal_mag;
    uint8_t xcvr_flags = (usbl_use_ahrs) | (xcvr_posflt_enable << 1) |
                         (xcvr_tx_msgctrl << 3) | (xcvr_usbl_msgs << 5) |
                         (xcvr_fix_msgs << 6) | (xcvr_diag_msgs << 7);

    new_settings.status_flags = status_flags;
    new_settings.enviromental_flags = enviromental_flags;
    new_settings.ahrs_flags = ahrs_flags;
    new_settings.xcvr_flags = xcvr_flags;
    new_settings.xcvr_range_tmo = static_cast<uint16_t>(xcvr_range_tmo);
    new_settings.xcvr_resp_time = static_cast<uint16_t>(xcvr_resp_time.toMilliseconds());
    new_settings.xcvr_posflt_tmo = static_cast<uint16_t>(xcvr_posflt_tmo.toSeconds());
    new_settings.xcvr_beacon_id = xcvr_beacon_id;
    mDriver->writeSettings(new_settings);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.
bool Task::configureHook()
{
    if (_status_mode.get() == protocol::STATUS_MODE_MANUAL) {
        LOG_ERROR_S << "STATUS_MODE_MANUAL not allowed as a status mode, choose one of "
                    << "the periodic status modes";
        return false;
    }

    std::unique_ptr<usbl_seatrac::Driver> driver(new Driver());
    iodrivers_base::ConfigureGuard guard(this);
    if (!_io_port.get().empty()) {
        driver->openURI(_io_port.get());
    }
    setDriver(driver.get());

    if (!TaskBase::configureHook()) {
        return false;
    }

    mDestinationId = _destination_id.get();
    mMsgType = _msg_type.get();
    m_safe_operational_pressure = _safe_operational_pressure.get();

    mDriver = move(driver);
    guard.commit();

    configureUSBLSettings(_xcvr_beacon_id.get(),
        _xcvr_tx_msgctrl.get(),
        _status_mode.get(),
        _xcvr_resp_time.get(),
        _xcvr_posflt_tmo.get(),
        _auto_vos.get(),
        _auto_pressure_ofs.get(),
        _auto_cal_mag.get(),
        _usbl_use_ahrs.get(),
        _xcvr_posflt_enable.get(),
        _xcvr_usbl_msgs.get(),
        _xcvr_fix_msgs.get(),
        _xcvr_diag_msgs.get(),
        _xcvr_range_tmo.get());

    mDriver->writeStatusConfig(0, protocol::STATUS_MODE_MANUAL);
    return true;
}

bool Task::startHook()
{
    if (!TaskBase::startHook()) {
        return false;
    }

    // Doing this here instead of startHook is a detail, but it helps with unit testing
    mDriver->writeStatusConfig(protocol::STATUS_ENVIRONMENT | protocol::STATUS_ATTITUDE,
        _status_mode.get());

    Timeout status_timeout(Time::fromSeconds(3));
    while (mDriver->process() != Driver::UPDATE_STATUS) {
        if (status_timeout.elapsed()) {
            LOG_ERROR_S << "Did not receive a status message in 3s";
            return false;
        }
    }

    auto status = mDriver->getLastReceivedStatus();
    outputStatusData(status);

    mPingInFlight = false;
    if (isPressureSafe(status)) {
        writePingRequestIfPossible();
    }

    m_position_mode = _position_mode.get();
    m_track_count = _track_count.get();
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::writePingRequestIfPossible()
{
    if (state() == UNSAFE_WORKING_PRESSURE) {
        return;
    }
    else if (mPingInFlight) {
        return;
    }

    if (m_position_mode == POSITION_MODE_PING) {
        mDriver->writePingRequest(_destination_id.get(), _msg_type.get());
    }
    else if (m_position_mode == POSITION_MODE_TRACK) {
        mDriver->writeTrackRequest(_destination_id.get(), m_track_count);
    }
    mPingInFlight = true;
}

void Task::processIO()
{
    auto update = mDriver->process();
    if (update & Driver::UPDATE_STATUS) {
        outputStatusData(mDriver->getLastReceivedStatus());
    }
    if (update & Driver::UPDATE_PING_RESULT) {
        outputPingResultData(mDriver->getLastReceivedPingResult());
        mPingInFlight = false;
    }
    if (update & Driver::UPDATE_TRACK_RESULT) {
        auto data = mDriver->getLastReceivedTrackResult();
        outputTrackResultData(data);

        if (data.flag == ERROR || data.response.response_no == m_track_count) {
            mPingInFlight = false;
        }
    }

    updateWorkingPressureState(mDriver->getLastReceivedStatus());
    writePingRequestIfPossible();
}

void Task::outputStatusData(Status const& status)
{
    float pressure_bar = static_cast<float>(status.environment.pressure) / 1000;

    samples::RigidBodyState rbs_reference;
    rbs_reference.time = base::Time::now();
    rbs_reference.position = Eigen::Vector3d(NAN, NAN, -pressure_bar * 10);
    rbs_reference.orientation = convertToOrientationQuaterniond(status);
    _local2nwu_orientation_with_z.write(rbs_reference);
}

void Task::outputPingResultData(PingResult const& result)
{
    auto ping = result;
    ping.timestamp = base::Time::now();
    _ping_result.write(ping);

    if (ping.flag == 1) {
        auto rbs = convertToPositionRBS(ping.timestamp, ping.response.acoustic_fix);
        _remote2local_position.write(rbs);
    }
}

void Task::outputTrackResultData(TrackResult const& result)
{
    auto track = result;
    track.timestamp = base::Time::now();
    _track_result.write(track);

    if (track.flag == 1) {
        auto rbs = convertToPositionRBS(track.timestamp, track.response.acoustic_fix);
        _remote2local_position.write(rbs);
    }
}

void Task::stopHook()
{
    TaskBase::stopHook();

    mDriver->writeStatusConfig(0, protocol::STATUS_MODE_MANUAL);
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    mDriver.reset();
}

bool Task::isPressureSafe(Status const& status) const
{
    if (base::isUnset(m_safe_operational_pressure.toBar())) {
        return true;
    }

    float pressure_bar = static_cast<float>(status.environment.pressure) / 1000;
    return pressure_bar > m_safe_operational_pressure.toBar();
}

void Task::updateWorkingPressureState(Status const& status)
{
    auto safe = isPressureSafe(status);
    if (safe && state() == UNSAFE_WORKING_PRESSURE) {
        recover();
    }
    else if (!safe && state() != UNSAFE_WORKING_PRESSURE) {
        error(UNSAFE_WORKING_PRESSURE);
    }
}
