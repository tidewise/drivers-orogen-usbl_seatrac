/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <iodrivers_base/ConfigureGuard.hpp>

using namespace usbl_seatrac;
using base::samples::Pressure;
using base::samples::RigidBodyState;

Task::Task(std::string const& name)
    : TaskBase(name)
{
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

static RigidBodyState convertToPositionRBS(PingStatus const& data)
{
    RigidBodyState rbs;

    rbs.time = data.timestamp;
    rbs.position = Eigen::Vector3d(data.response.acoustic_fix.position.north / 10.0,
        data.response.acoustic_fix.position.east / 10.0,
        data.response.acoustic_fix.position.depth / 10.0);
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
    protocol::Settings new_settings = mDriver->getSettingsProtocol();

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
    mDriver->setSettingsProtocol(new_settings);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.
bool Task::configureHook()
{

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
    m_orientation_output_flag = _orientation_output_flag.get();
    m_ping_refresh_period = _ping_refresh_period.get();

    if (m_orientation_output_flag &&
        MINIMUM_PING_STATUS_REFRESH_TIME > m_ping_refresh_period) {
        LOG_ERROR_S << "If an orientation is required, because of hardware limitations, "
                       "the minimum ping refresh time is : "
                    << MINIMUM_PING_STATUS_REFRESH_TIME << std::endl;
        return false;
    }

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

    return true;
}

bool Task::startHook()
{
    if (!TaskBase::startHook()) {
        return false;
    }
    return true;
}

void Task::updateHook()
{
    Status status = mDriver->autoStatus();
    RigidBodyState rbs_reference;
    // Write the local usbl depth
    rbs_reference.time = status.timestamp;
    rbs_reference.position = Eigen::Vector3d(NAN, NAN, status.environment.depth / 10.);
    // Write the local usbl orientation
    if (m_orientation_output_flag) {
        rbs_reference.orientation = convertToOrientationQuaterniond(status);
    }

    _local2nwu_orientation_with_z.write(rbs_reference);

    if (base::Time::now() - m_previous_ping_refresh_time > m_ping_refresh_period) {
        m_previous_ping_refresh_time = base::Time::now();

        PingStatus ping = mDriver->Ping(mDestinationId, mMsgType);
        ping.timestamp = base::Time::now();
        _ping_status.write(ping);

        if (ping.flag == 1) {
            auto rbs = convertToPositionRBS(ping);
            _remote2local_position.write(rbs);
        }
    }

    TaskBase::updateHook();
}

void Task::processIO()
{
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    mDriver.reset();
}
