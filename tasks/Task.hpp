/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef USBL_SEATRAC_TASK_TASK_HPP
#define USBL_SEATRAC_TASK_TASK_HPP

#include "usbl_seatrac/TaskBase.hpp"
#include <base/samples/Pressure.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <usbl_seatrac/Driver.hpp>

/**
 * The minimum ping refresh rate in case it will need the local usbl's orientation
 * information
 */
static const base::Time MINIMUM_PING_STATUS_REFRESH_TIME = base::Time::fromSeconds(2.4);

namespace usbl_seatrac {

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine
to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These
interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the
associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the usbl_seatrac namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','usbl_seatrac::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix
argument.
     */
    class Task : public TaskBase {
        friend class TaskBase;

    protected:
        std::unique_ptr<usbl_seatrac::Driver> mDriver;

    private:
        usbl_seatrac::protocol::BeaconIdentificationCode mDestinationId;
        usbl_seatrac::protocol::AcousticMessageType mMsgType;
        /**
         * The ping's refresh period
         */
        base::Time m_ping_refresh_period;
        /**
         * The previous time where a ping was done
         */
        base::Time m_previous_ping_refresh_time;
        /**
         * If it outputs the orientation
         */
        bool m_orientation_output_flag;

        void processIO();

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it
         * identifiable via nameservices. \param initial_state The initial TaskState of
         * the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "usbl_seatrac::Task");

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** This configures the USBL settings using get and set protocols with the usbl
         * based on the given proprierty parameters
         */
        void configureUSBLSettings(
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
            float xcvr_range_tmo);
    };
}

#endif
