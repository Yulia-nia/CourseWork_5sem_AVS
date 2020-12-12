/**
 * late_alu.cpp - Simulation of execute units
 * Copyright 2015-2018 MIPT-MIPS
 */

#include <infra/config/config.h>

#include "late_alu.h"

namespace config {
    const PredicatedValue<uint64> long_late_alu_latency = { "long-late-alu-latency", 3, "Latency of long arithmetic logic unit",
                                                            [](uint64 val) { return val >= 2 && val < 64; } };
} // namespace config

template <typename FuncInstr>
Late_alu<FuncInstr>::Late_alu( Module* parent) : Module( parent, "late_alu")
        , last_execution_stage_latency( Latency( config::long_late_alu_latency - 1))
{
    wp_writeback_datapath = make_write_port<Instr>("LATE_ALU_2_WRITEBACK", Port::BW);

    rp_datapath = make_read_port<Instr>("EXECUTE_2_LATE_ALU", Port::LATENCY);
    rp_trap = make_read_port<bool>("WRITEBACK_2_ALL_FLUSH", Port::LATENCY);

    wp_long_latency_execution_unit = make_write_port<Instr>("LATE_ALU_2_EXECUTE_LONG_LATENCY", Port::BW);
    rp_long_latency_execution_unit = make_read_port<Instr>("LATE_ALU_2_EXECUTE_LONG_LATENCY", last_execution_stage_latency);

    rp_flush = make_read_port<bool>("BRANCH_2_ALL_FLUSH", Port::LATENCY);

    rps_bypass[0].command_port = make_read_port<BypassCommand<Register>>("DECODE_2_LATE_ALU_SRC1_COMMAND", Port::LATENCY +1_lt);
    rps_bypass[1].command_port = make_read_port<BypassCommand<Register>>("DECODE_2_LATE_ALU_SRC2_COMMAND", Port::LATENCY +1_lt);

    wp_bypass = make_write_port<InstructionOutput>("LATE_ALU_2_EXECUTE_BYPASS", Port::BW);

    rps_bypass[0].data_ports[0] = make_read_port<InstructionOutput>("EXECUTE_2_EXECUTE_BYPASS", Port::LATENCY);
    rps_bypass[1].data_ports[0] = make_read_port<InstructionOutput>("EXECUTE_2_EXECUTE_BYPASS", Port::LATENCY);

    rps_bypass[0].data_ports[1] = make_read_port<InstructionOutput>("EXECUTE_COMPLEX_ALU_2_EXECUTE_BYPASS", Port::LATENCY);
    rps_bypass[1].data_ports[1] = make_read_port<InstructionOutput>("EXECUTE_COMPLEX_ALU_2_EXECUTE_BYPASS", Port::LATENCY);

    rps_bypass[0].data_ports[2] = make_read_port<InstructionOutput>("MEMORY_2_EXECUTE_BYPASS", Port::LATENCY);
    rps_bypass[1].data_ports[2] = make_read_port<InstructionOutput>("MEMORY_2_EXECUTE_BYPASS", Port::LATENCY);

    rps_bypass[0].data_ports[3] = make_read_port<InstructionOutput>("WRITEBACK_2_EXECUTE_BYPASS", Port::LATENCY);
    rps_bypass[1].data_ports[3] = make_read_port<InstructionOutput>("WRITEBACK_2_EXECUTE_BYPASS", Port::LATENCY);

    rps_bypass[0].data_ports[4] = make_read_port<InstructionOutput>("BRANCH_2_EXECUTE_BYPASS", Port::LATENCY);
    rps_bypass[1].data_ports[4] = make_read_port<InstructionOutput>("BRANCH_2_EXECUTE_BYPASS", Port::LATENCY);

    rps_bypass[0].data_ports[5] = make_read_port<InstructionOutput>("LATE_ALU_2_EXECUTE_BYPASS", Port::LATENCY);
    rps_bypass[1].data_ports[5] = make_read_port<InstructionOutput>("LATE_ALU_2_EXECUTE_BYPASS", Port::LATENCY);
}

template <typename FuncInstr>
void Late_alu<FuncInstr>::clock( Cycle cycle)
{
    sout << "exelate cycle " << std::dec << cycle << ": ";

    const bool is_flush = ( rp_flush->is_ready( cycle) && rp_flush->read( cycle))
                          || ( rp_trap->is_ready( cycle) && rp_trap->read( cycle));

    // update information about mispredictions
    clock_saved_flush();

    // branch misprediction
    if (is_flush)
    {
        save_flush();
        sout << "flush\n";
        return;
    }

    /* check if there is something to process */
    if ( !rp_datapath->is_ready( cycle))
    {
        sout << "bubble\n";
        return;
    }

    auto instr = rp_datapath->read( cycle);

    auto src_index = 0;
    for ( auto& bypass_source : rps_bypass)
    {
        if (bypass_source.command_port->is_ready(cycle))
        {
            const auto bypass_direction = bypass_source.command_port->read(cycle).get_ready();
            auto& port = bypass_source.data_ports.at(bypass_direction);
            RegisterUInt data{};
            while (port->is_ready(cycle))
                data = port->read(cycle)[0];
            instr.set_v_src(data, src_index);
        }
        else
        {
            rf->read_source(&instr, src_index);
        }
        ++src_index;
    }

    // perform execution
    instr.execute();

    // log
    sout << instr << std::endl;

    if (instr.get_alu_number() == 0)
    {
        if ( instr.is_long_arithmetic()) {
            wp_long_latency_execution_unit->write( std::move( instr), cycle);
        }
        else
        {
            wp_bypass->write( instr.get_v_dst(), cycle);

            if( instr.is_jump()) {
                wp_branch_datapath->write(std::move(instr), cycle);
            }
            else
            {
                wp_writeback_datapath->write( std::move( instr), cycle);
            }
        }
    }
}


#include <mips/mips.h>
#include <risc_v/risc_v.h>

template class Late_alu<BaseMIPSInstr<uint32>>;
template class Late_alu<BaseMIPSInstr<uint64>>;
template class Late_alu<RISCVInstr<uint32>>;
template class Late_alu<RISCVInstr<uint64>>;
template class Late_alu<RISCVInstr<uint128>>;


