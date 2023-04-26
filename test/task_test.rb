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

    attr_reader :task, :raw_io

    before do
        @task = syskit_deploy(OroGen.usbl_seatrac.Task.deployed_as("usbl_test"))
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
        @task.properties.status_mode = 0x02

        @raw_io = setup_iodrivers_base_with_ports(@task, configure_and_start: false)
    end

    describe "configuration" do
        it "updates the settings for the parameters it controls" do
            set_cmd = usbl_configure_and_start(raw_io, task)
            # Verify the structure of set_cmd
            expected_packet = raw_packet_from_s("#16023F0D0D000000000000FA01A8C00000FFFF"\
                "0101A8C00101A8C0A41F014F0000005E012B3C00FCFEFFFEE4FEE900F800F60001405C2"\
                "EC1D4B358C22B3C4543F2C5833FE27C7E3FB5257A3F777E8942AE22D140000000000000"\
                "000000000000630F2C010A00000000000000030A3C7680\r\n")
            assert_equal(set_cmd.data, expected_packet.data)
        end
    end

    describe "while running" do
        before do
            usbl_configure_and_start(raw_io, task)
        end

        it "interprets a ping response message from the device" do
            packet1 = raw_packet_from_s("$4000028015\r\n")
            packet2 = raw_packet_from_s("$42020F07022402130007070101010101010101010101" \
                "01010101010101010101010101010101010101010101010101010F78\r\n")
            response =
                expect_execution \
                    { syskit_write @raw_io.out_port, packet1, packet2 }
                .to do
                    [
                        have_one_new_sample(task.remote2local_position_port),
                        have_one_new_sample(task.local2nwu_orientation_with_z_port),
                        have_one_new_sample(task.ping_status_port)
                    ]
                end

            assert_in_delta(25.7, response[0].position.x, 1)
            assert_in_delta(25.7, response[0].position.y, 1)
            assert_in_delta(25.7, response[0].position.z, 1)

            assert(response[1].position.x.nan?)
            assert(response[1].position.y.nan?)
            assert_equal(25.7, response[1].position.z)
            assert_in_delta(
                (179.9 * (3.14159265 / 180)), response[1].orientation.roll, 0.1
            )
            assert_in_delta(
                (1.9 * (3.14159265 / 180)), response[1].orientation.pitch, 0.1
            )
            assert_in_delta(
                (54.8 * (3.14159265 / 180)), response[1].orientation.yaw, 0.1
            )

            assert_equal(257, response[2].response.acoustic_fix.position.north)
            assert_equal(257, response[2].response.acoustic_fix.position.east)
            assert_equal(257, response[2].response.acoustic_fix.position.depth)
            assert_equal(548, response[2].response.acoustic_fix.attitude_yaw)
            assert_equal(19, response[2].response.acoustic_fix.attitude_pitch)
            assert_equal(1799, response[2].response.acoustic_fix.attitude_roll)
        end

        it "interprets a ping error message from the device" do
            packet1 = raw_packet_from_s("$4000028015\r\n")
            packet2 = raw_packet_from_s("$43340266D5\r\n")
            expect_execution { syskit_write @raw_io.out_port, packet1, packet2 }
                .to { have_no_new_sample task.remote2local_position_port, at_least_during: 0.5 }
        end
    end

    # rubocop: disable Metrics/AbcSize
    def usbl_configure_and_start(raw_io, task)
        get_reply = raw_packet_from_s("$15023F0D0D000000000000FA01A8C00000FFFF0101A8C0" \
            "0101A8C0A41F014F0000005E012B3C00FCFEFFFEE4FEE900F800F600FF405C2EC1D4B358C" \
            "22B3C4543F2C5833FE27C7E3FB5257A3F777E8942AE22D140000000000000000000000000" \
            "630F64000A00000000000000030A3C3DB9\r\n")
        set_reply = raw_packet_from_s("$16000E60\r\n")
        # Prepare the sample you want to send
        writer = syskit_create_writer raw_io.out_port
        get_cmd = expect_execution.scheduler(true)
                                  .join_all_waiting_work(false)
                                  .to { have_one_new_sample task.io_raw_out_port }
        # check that get_cmd is actually a get command
        get_request = raw_packet_from_s("#15C1CF\r\n")
        assert_equal(get_request.data, get_cmd.data)
        set_cmd = expect_execution { writer.write get_reply }
                  .join_all_waiting_work(false)
                  .to { have_one_new_sample task.io_raw_out_port }
        execute { writer.write set_reply }
        syskit_configure_and_start(@task)
        set_cmd
    end
    # rubocop: enable Metrics/AbcSize
end
