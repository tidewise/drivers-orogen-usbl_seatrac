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

static RigidBodyState convertToRBS(PingStatus const& data)
{
    Eigen::Quaterniond attitude =
        Eigen::Quaterniond(Eigen::AngleAxisd(-data.response.acoustic_fix.attitude_yaw / 10. / 180.0 * M_PI + M_PI,
            Eigen::Vector3d::UnitZ()));
    RigidBodyState rbs;

    rbs.time = data.timestamp;
    rbs.position = Eigen::Vector3d(-data.response.acoustic_fix.position.north / 10.0,
        data.response.acoustic_fix.position.east / 10.0,
        -data.response.acoustic_fix.position.depth / 10.0);
    rbs.position = attitude * rbs.position;
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
