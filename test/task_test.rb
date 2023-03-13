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
        @task = syskit_deploy(
            OroGen.usbl_seatrac.Task.deployed_as("usbl_test")
        )
        @task.properties.destination_id = 0x02
        @task.properties.msg_type = 0x06
        @raw_io = setup_iodrivers_base_with_ports @task
    end

    it "interprets a ping response message from the device" do
        packet1 = raw_packet_from_s("$1001B200000000000000010001000100000001000000010" \
            "0B429\r\n$")
        packet2 = raw_packet_from_s("$4000028015\r\n")
        packet3 = raw_packet_from_s("$42020F07020101010101010101010101010101010101010" \
            "101010101010101010101010101010101010101010101010101B0\r\n")
        response =
            expect_execution { syskit_write @raw_io.out_port, packet1, packet2, packet3 }
            .to do
                [
                    have_one_new_sample(task.local_pose_port),
                    have_one_new_sample(task.global_pose_port),
                    have_one_new_sample(task.ping_status_port)
                ]
            end
        assert_equal(25.7, response[0].position.x)
        assert_equal(25.7, response[0].position.y)
        assert_equal(25.7, response[0].position.z)

        assert_in_delta(21.96, response[1].position.x, 1)
        assert_in_delta(23.12, response[1].position.y, 1)
        assert_in_delta(31.04, response[1].position.z, 1)

        assert_equal(257, response[2].response.acoustic_fix.position.north)
        assert_equal(257, response[2].response.acoustic_fix.position.east)
        assert_equal(257, response[2].response.acoustic_fix.position.depth)
        assert_equal(257, response[2].response.acoustic_fix.attitude_yaw)
        assert_equal(257, response[2].response.acoustic_fix.attitude_pitch)
        assert_equal(257, response[2].response.acoustic_fix.attitude_roll)
    end

    it "interprets a ping error message from the device" do
        packet1 = raw_packet_from_s("$1001B200000000000000010001000100000001000000010" \
            "0B429\r\n$")
        packet2 = raw_packet_from_s("$4000028015\r\n")
        packet3 = raw_packet_from_s("$43340266D5\r\n")
        response =
            expect_execution { syskit_write @raw_io.out_port, packet1, packet2, packet3 }
            .to { have_no_new_sample task.local_pose_port, at_least_during: 0.5 }
    end

    it "stop if there is no status packet from the device" do
        packet1 = raw_packet_from_s("$4000028015\r\n")
        packet2 = raw_packet_from_s("$42020F07020101010101010101010101010101010101010" \
            "101010101010101010101010101010101010101010101010101B0\r\n")
        response =
            expect_execution { syskit_write @raw_io.out_port, packet1, packet2}
            .to { have_no_new_sample task.local_pose_port, at_least_during: 0.5 }
    end
end
