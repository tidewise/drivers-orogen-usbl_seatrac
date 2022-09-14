/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <iodrivers_base/ConfigureGuard.hpp>

using namespace usbl_seatrac;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook() {

    std::unique_ptr<gps_ublox::Driver> driver(new Driver());
    iodrivers_base::ConfigureGuard guard(this);
    if (!_io_port.get().empty()) {
        driver->openURI(_io_port.get());
    }
    setDriver(driver.get());

    if (! TaskBase::configureHook()) {
        return false;
    }

    mDriver = move(driver);
    guard.commit();
    return true;
}

bool Task::startHook() {
    if (! TaskBase::startHook()) {
        return false;
    }
    return true;
}

void Task::updateHook() {
    TaskBase::updateHook();
}

void Task::errorHook() {
    TaskBase::errorHook();
}

void Task::stopHook() {
    TaskBase::stopHook();
}

void Task::cleanupHook() {
    TaskBase::cleanupHook();
    mDriver.reset();
}
