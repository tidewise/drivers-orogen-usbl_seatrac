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

static Eigen::Quaterniond convertToQuaterniond(PingStatus const& data)
{
    Eigen::Quaterniond attitude = Eigen::Quaterniond(
        Eigen::AngleAxisd(data.response.acoustic_fix.attitude_roll / 10. / 180.0 * M_PI,
            Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(data.response.acoustic_fix.attitude_pitch / 10. / 180.0 * M_PI,
            Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(data.response.acoustic_fix.attitude_yaw / 10. / 180.0 * M_PI,
            Eigen::Vector3d::UnitZ()));
    return attitude;
}

static RigidBodyState convertToRBS(PingStatus const& data)
{
    RigidBodyState rbs;

    rbs.time = data.timestamp;
    rbs.position = Eigen::Vector3d(data.response.acoustic_fix.position.north / 10.0,
        data.response.acoustic_fix.position.east / 10.0,
        data.response.acoustic_fix.position.depth / 10.0);
    return rbs;
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

    mDestinationId = _destination_id.get();
    mMsgType = _msg_type.get();

    if (!TaskBase::configureHook()) {
        return false;
    }

    mDriver = move(driver);
    guard.commit();

    // Set initial settings
    message::Frame old_settings = mDriver->getSettings();
    SettingsGet get = parseGetSettings(old_settings.payload);
    protocol::Settings new_settings = get.settings;

    uint8_t status_flags = _status_mode.get();
    uint8_t enviromental_flags = _auto_vos.get() | (_auto_pressure_ofs.get() << 1);
    uint8_t ahrs_flags = _auto_cal_mag.get();
    uint8_t xcvr_flags = (_usbl_use_ahrs.get()) | (_xcvr_posflt_enable.get() << 1) |
                         (_xcvr_tx_msgctrl.get() << 3) | (_xcvr_usbl_msgs.get() << 5) |
                         (_xcvr_fix_msgs.get() << 6) | (_xcvr_diag_msgs.get() << 7);

    new_settings.status_flags = status_flags;
    new_settings.enviromental_flags = enviromental_flags;
    new_settings.ahrs_flags = ahrs_flags;
    new_settings.xcvr_flags = xcvr_flags;
    new_settings.xcvr_range_tmo = static_cast<uint16_t>(_xcvr_range_tmo.get());
    new_settings.xcvr_resp_time =
        static_cast<uint16_t>(_xcvr_resp_time.get().toMilliseconds());
    new_settings.xcvr_posflt_tmo =
        static_cast<uint16_t>(_xcvr_posflt_tmo.get().toSeconds());
    new_settings.xcvr_beacon_id = _xcvr_beacon_id.get();
    mDriver->setSettings(new_settings);
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
    Pressure pressure;
    pressure.time = base::Time::now();
    pressure = pressure.fromBar(pressure.time,
        static_cast<float>(status.environment.pressure) / 1000);
    _local_pressure.write(pressure);

    PingStatus ping = mDriver->Ping(mDestinationId, mMsgType);
    ping.timestamp = base::Time::now();
    _ping_status.write(ping);
    if (ping.flag == 1) {
        auto rbs = convertToRBS(ping);
        auto attitude = convertToQuaterniond(ping);
        _local_attitude.write(attitude);
        rbs.position = attitude * rbs.position;
        _pose.write(rbs);
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
