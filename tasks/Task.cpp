/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <iodrivers_base/ConfigureGuard.hpp>

using namespace usbl_seatrac;
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
    RigidBodyState rbs;

    rbs.time = data.timestamp;
    rbs.position = Eigen::Vector3d(data.response.acoustic_fix.position.north,
        -data.response.acoustic_fix.position.east,
        -data.response.acoustic_fix.position.depth);
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
    PingStatus status = mDriver->Ping(mDestinationId, mMsgType);
    status.timestamp = base::Time::now();
    _ping_status.write(status);
    if (status.flag == 1) {
        auto rbs = convertToRBS(status);
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
