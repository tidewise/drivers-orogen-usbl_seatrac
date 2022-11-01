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
        packet1 = raw_packet_from_s("$4000028015\r\n")
        packet2 = raw_packet_from_s("$42020F01020101010101010101010101010101010101010" \
            "10101010101010101010101010101010101010101010101016413\r\n")
        response =
            expect_execution { syskit_write @raw_io.out_port, packet1, packet2 }
            .to { have_one_new_sample task.pose_port }
        assert_equal(257, response.position.x)
        assert_equal(-257, response.position.y)
        assert_equal(-257, response.position.z)
    end

    it "interprets a ping error message from the device" do
        packet1 = raw_packet_from_s("$4000028015\r\n")
        packet2 = raw_packet_from_s("$43340266D5\r\n")
        response =
            expect_execution { syskit_write @raw_io.out_port, packet1, packet2 }
            .to { have_no_new_sample task.pose_port, at_least_during: 0.5 }
    end
end
