# frozen_string_literal: true

using_task_library "iodrivers_base"
require "iodrivers_base/orogen_test_helpers"
using_task_library "usbl_seatrac"

import_types_from "base"
import_types_from "usbl_seatrac"
import_types_from "iodrivers_base"

describe OroGen.usbl_seatrac.Task do
    include IODriversBase::OroGenTestHelpers

    run_live

    attr_reader :task, :raw_io, :from_driver, :to_driver

    after do
        if task.running?
            expect_execution { task.stop! }.join_all_waiting_work(false).to_run

            status_cfg_set = usbl_handle_configuration_request("$12000CA0\r\n")
            assert_equal "#120000A005\r\n", status_cfg_set
            expect_execution.to_emit task.stop_event
        end
    end

    describe "configuration" do
        it "configures the usbl's data properly" do
            usbl_task_setup
            set_cmd = usbl_configure
            # Verify the structure of set_cmd
            expected = "#16023F0D0D000000000000FA01A8C00000FFFF"\
                "0101A8C00101A8C0A41F014F0000005E012B3C00FCFEFFFEE4FEE900F800F60001405C2"\
                "EC1D4B358C22B3C4543F2C5833FE27C7E3FB5257A3F777E8942AE22D140000000000000"\
                "000000000000630F2C010A00000000000000030A3C7680\r\n"
            assert_equal(expected, set_cmd)
        end
    end

    describe "at runtime" do
        before do
            usbl_task_setup
            usbl_configure_and_start
        end

        it "interprets a status message from the device" do
            status_message = raw_packet_from_s(
                "$103FA4E50F0000000000285D09010D00000001000000" \
                "133C2406EDFFF9F80EFF090100005EF9FE1AFFE1FEC400F3005301EDFF0200EA009B0" \
                "551FDC8FB16000000040070C8B6C018E2D24055687443A410BA4487451DC41C019CC4" \
                "0000B04100000000000080408C00\r\n"
            )

            response =
                expect_execution { syskit_write to_driver, status_message }
                .to_have_one_new_sample(task.local2nwu_orientation_with_z_port)

            assert(response.position.x.nan?)
            assert(response.position.y.nan?)
            assert_in_delta(-0.13, response.position.z, 1e-6)
            assert_in_delta(
                (-179.9 * (Math::PI / 180)), response.orientation.roll, 0.1
            )
            assert_in_delta(
                (-1.9 * (Math::PI / 180)), response.orientation.pitch, 0.1
            )
            assert_in_delta(
                (157.2 * (Math::PI / 180)), response.orientation.yaw, 0.1
            )
        end

        it "interprets a ping response success message from the device and requests a new ping" do
            ping_message = raw_packet_from_s(
                "$42020F07020101010101010101010101010101010101" \
                "010101010101010101010101010101010101010101010101010101B0\r\n"
            )

            response =
                expect_execution { syskit_write to_driver, ping_message }
                .to do
                    [
                        have_one_new_sample(task.remote2local_position_port),
                        have_one_new_sample(task.ping_result_port)
                    ]
                end

            ping_send = usbl_handle_configuration_request("$400001C014\r\n")
            assert_equal "#40020680B6\r\n", ping_send

            assert_in_delta(25.7, response[0].position.x, 1)
            assert_in_delta(25.7, response[0].position.y, 1)
            assert_in_delta(25.7, response[0].position.z, 1)

            assert_equal(257, response[1].response.acoustic_fix.position.north)
            assert_equal(257, response[1].response.acoustic_fix.position.east)
            assert_equal(257, response[1].response.acoustic_fix.position.depth)
            assert_equal(257, response[1].response.acoustic_fix.attitude_yaw)
            assert_equal(257, response[1].response.acoustic_fix.attitude_pitch)
            assert_equal(257, response[1].response.acoustic_fix.attitude_roll)
        end

        it "interprets a ping error message from the device" do
            ping_error = raw_packet_from_s("$43340266D5\r\n")
            expect_execution { syskit_write @to_driver, ping_error }
                .to do
                    [
                        have_no_new_sample(
                            task.remote2local_position_port,
                            at_least_during: 0.5
                        ),
                        have_one_new_sample(task.ping_result_port)
                    ]
                end

            ping_send = usbl_handle_configuration_request("$400001C014\r\n")
            assert_equal "#40020680B6\r\n", ping_send
        end

        it "times out if there are no packets coming from the device" do
            # Wait for the component's stopHook, which attempts to disable
            # periodic status messages
            status_cfg_set = usbl_handle_configuration_request("$12000CA0\r\n")
            assert_equal "#120000A005\r\n", status_cfg_set
            expect_execution.to_emit task.io_timeout_event
        end
    end

    describe "behavior regarding the safe working pressure parameter" do
        before do
            usbl_task_setup
        end

        it "does not ping at all if the pressure is below the safe threshold" do
            task.properties.safe_operational_pressure = { pascal: 1_400 }
            usbl_configure_and_start(expect_first_ping: false)
        end

        it "pings if the pressure is above the safe threshold" do
            task.properties.safe_operational_pressure = { pascal: 1_200 }
            usbl_configure_and_start(expect_first_ping: true)

            ping_message = raw_packet_from_s(
                "$42020F07020101010101010101010101010101010101" \
                "010101010101010101010101010101010101010101010101010101B0\r\n"
            )
            expect_execution { syskit_write to_driver, ping_message }
                .to do
                    have_one_new_sample(task.remote2local_position_port)
                    have_one_new_sample(task.ping_result_port)
                end

            ping_send = usbl_handle_configuration_request("$400001C014\r\n")
            assert_equal "#40020680B6\r\n", ping_send

            refute task.unsafe_working_pressure_event.emitted?
        end

        it "stops pinging if the pressure gets below the safe threshold" do
            task.properties.safe_operational_pressure = { pascal: 1_500 }
            usbl_configure_and_start(
                expect_first_ping: true,
                status_message: STATUS_WITH_20_MILLIBAR_PRESSURE
            )

            message = raw_packet_from_s(STATUS_WITH_13_MILLIBAR_PRESSURE)
            expect_execution { syskit_write to_driver, message }
                .to do
                    have_one_new_sample(task.local2nwu_orientation_with_z_port)
                    emit task.unsafe_working_pressure_event
                end

            ping_message = raw_packet_from_s(
                "$42020F07020101010101010101010101010101010101" \
                "010101010101010101010101010101010101010101010101010101B0\r\n"
            )
            expect_execution { syskit_write to_driver, ping_message }
                .to_have_no_new_sample from_driver, at_least_during: 0.5
        end

        it "starts pinging if the pressure gets above the safe threshold" do
            task.properties.safe_operational_pressure = { pascal: 1_500 }
            usbl_configure_and_start(expect_first_ping: false)

            message = raw_packet_from_s(STATUS_WITH_13_MILLIBAR_PRESSURE)
            to_driver.write(message)
            expect_execution.to_emit task.unsafe_working_pressure_event

            message = raw_packet_from_s(STATUS_WITH_20_MILLIBAR_PRESSURE)
            expect_execution { to_driver.write(message) }
                .to_emit task.running_event
            ping_send = usbl_handle_configuration_request("$400001C014\r\n")
            assert_equal "#40020680B6\r\n", ping_send
        end

        it "does not ping again after an unsafe-safe transition if a ping is " \
           "already in progress" do
            task.properties.safe_operational_pressure = { pascal: 1_500 }
            usbl_configure_and_start(expect_first_ping: false)

            message = raw_packet_from_s(STATUS_WITH_20_MILLIBAR_PRESSURE)
            to_driver.write(message)
            usbl_handle_configuration_request("$400001C014\r\n")

            message = raw_packet_from_s(STATUS_WITH_13_MILLIBAR_PRESSURE)
            expect_execution { to_driver.write(message) }
                .to_emit task.unsafe_working_pressure_event
            message = raw_packet_from_s(STATUS_WITH_20_MILLIBAR_PRESSURE)
            expect_execution { to_driver.write(message) }
                .to do
                    emit task.running_event
                    have_no_new_sample from_driver
                end

            ping_message = raw_packet_from_s(
                "$42020F07020101010101010101010101010101010101" \
                "010101010101010101010101010101010101010101010101010101B0\r\n"
            )
            syskit_write to_driver, ping_message
            ping_send = usbl_handle_configuration_request("$400001C014\r\n")
            assert_equal "#40020680B6\r\n", ping_send
        end
    end

    def usbl_task_setup(safe_operational_pressure = Float::NAN)
        @task = syskit_deploy(OroGen.usbl_seatrac.Task.deployed_as("usbl_test"))
        @task.properties.io_wait_timeout = Time.at(1)
        @task.properties.destination_id = 0x02
        @task.properties.msg_type = 0x06
        @task.properties.xcvr_beacon_id = 0x0F
        @task.properties.auto_pressure_ofs = false
        @task.properties.auto_vos = true
        @task.properties.auto_cal_mag = false
        @task.properties.xcvr_diag_msgs = false
        @task.properties.xcvr_fix_msgs = true
        @task.properties.xcvr_usbl_msgs = true
        @task.properties.xcvr_tx_msgctrl = 0b0
        @task.properties.xcvr_posflt_enable = true
        @task.properties.usbl_use_ahrs = true
        @task.properties.xcvr_range_tmo = 300
        @task.properties.xcvr_resp_time = Time.at(0.01)
        @task.properties.xcvr_posflt_tmo = Time.at(60)
        @task.properties.status_mode = "STATUS_MODE_2HZ5"
        @task.properties.safe_operational_pressure = { pascal: safe_operational_pressure }
        @raw_io = setup_iodrivers_base_with_ports(@task, configure_and_start: false)
        @from_driver = syskit_create_reader @raw_io.in_port, type: :buffer, size: 20
        @to_driver = syskit_create_writer @raw_io.out_port, type: :buffer, size: 20
    end

    def usbl_configure
        get_reply = "$15023F0D0D000000000000FA01A8C00000FFFF0101A8C0" \
            "0101A8C0A41F014F0000005E012B3C00FCFEFFFEE4FEE900F800F600FF405C2EC1D4B358C" \
            "22B3C4543F2C5833FE27C7E3FB5257A3F777E8942AE22D140000000000000000000000000" \
            "630F64000A00000000000000030A3C3DB9\r\n"

        get = usbl_handle_configuration_request(get_reply)
        assert_equal("#15C1CF\r\n", get)

        set_cmd = usbl_handle_configuration_request("$16000E60\r\n")
        assert set_cmd.start_with?("#16")

        status_cfg_set = usbl_handle_configuration_request("$12000CA0\r\n")
        assert_equal "#120000A005\r\n", status_cfg_set

        syskit_configure(task)
        set_cmd
    end

    def usbl_configure_and_start(
        expect_first_ping: true, status_message: STATUS_WITH_13_MILLIBAR_PRESSURE
    )
        usbl_configure

        expect_execution { task.start! }.join_all_waiting_work(false).to_run

        status_cfg_set = usbl_handle_configuration_request("$12000CA0\r\n")
        assert_equal "#1203022134\r\n", status_cfg_set
        to_driver.write raw_packet_from_s(status_message)

        if expect_first_ping
            ping_send = usbl_handle_configuration_request("$400001C014\r\n")
            assert_equal "#40020680B6\r\n", ping_send
        end

        expect_execution.to_emit task.start_event
    end

    def usbl_handle_configuration_request(reply, &predicates)
        msg =
            expect_execution
            .scheduler(true)
            .join_all_waiting_work(false)
            .to_have_one_new_sample(from_driver)
        expect_execution { to_driver.write raw_packet_from_s(reply) }
            .join_all_waiting_work(false).to do
                yield(self) if block_given?
            end

        msg.data.to_byte_array[8..-1]
    end

    STATUS_WITH_13_MILLIBAR_PRESSURE =
        "$103FA4E50F0000000000285D09010D00000001000000" \
        "133C2406EDFFF9F80EFF090100005EF9FE1AFFE1FEC400F3005301EDFF0200EA009B0" \
        "551FDC8FB16000000040070C8B6C018E2D24055687443A410BA4487451DC41C019CC4" \
        "0000B04100000000000080408C00\r\n"
    STATUS_WITH_20_MILLIBAR_PRESSURE =
        "$103FA4E50F0000000000285D09012000000001000000" \
        "133C2406EDFFF9F80EFF090100005EF9FE1AFFE1FEC400F3005301EDFF0200EA009B0" \
        "551FDC8FB16000000040070C8B6C018E2D24055687443A410BA4487451DC41C019CC4" \
        "0000B0410000000000008040703F\r\n"
end
