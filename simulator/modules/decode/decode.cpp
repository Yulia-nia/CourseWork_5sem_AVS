/**
 * decode.cpp - simulation of decode stage
 * Copyright 2015-2018 MIPT-MIPS
 */

#include "decode.h"

#include <modules/execute/execute.h>
#include <modules/late_alu/late_alu.h>

template <typename FuncInstr>
Decode<FuncInstr>::Decode( Module* parent) : Module( parent, "decode")
{
    bypassing_unit = std::make_unique<BypassingUnit>( config::long_alu_latency);

    rp_datapath = make_read_port<Instr>("FETCH_2_DECODE", Port::LATENCY);
    rp_stall_datapath = make_read_port<Instr>("DECODE_2_DECODE", Port::LATENCY);
    rp_flush = make_read_port<bool>("BRANCH_2_ALL_FLUSH", Port::LATENCY);
    rp_bypassing_unit_notify = make_read_port<Instr>("DECODE_2_BYPASSING_UNIT_NOTIFY", Port::LATENCY);
    rp_bypassing_unit_flush_notify = make_read_port<bool>("BRANCH_2_BYPASSING_UNIT_FLUSH_NOTIFY", Port::LATENCY);
    rp_flush_fetch = make_read_port<bool>("DECODE_2_FETCH_FLUSH", Port::LATENCY);
    rp_trap = make_read_port<bool>("WRITEBACK_2_ALL_FLUSH", Port::LATENCY);

    wp_datapath = make_write_port<Instr>("DECODE_2_EXECUTE", Port::BW);
    wp_stall_datapath = make_write_port<Instr>("DECODE_2_DECODE", Port::BW);
    wp_stall = make_write_port<bool>("DECODE_2_FETCH_STALL", Port::BW);
    wps_command[0] = make_write_port<BypassCommand<Register>>("DECODE_2_EXECUTE_SRC1_COMMAND", Port::BW);
    wps_command[1] = make_write_port<BypassCommand<Register>>("DECODE_2_EXECUTE_SRC2_COMMAND", Port::BW);
    wps_command_late_alu[0] = make_write_port<BypassCommand<Register>>("DECODE_2_LATE_ALU_SRC1_COMMAND", Port::BW);
    wps_command_late_alu[1] = make_write_port<BypassCommand<Register>>("DECODE_2_LATE_ALU_SRC2_COMMAND", Port::BW);
    wp_bypassing_unit_notify = make_write_port<Instr>("DECODE_2_BYPASSING_UNIT_NOTIFY", Port::BW);
    wp_flush_fetch = make_write_port<bool>("DECODE_2_FETCH_FLUSH", Port::BW);
    wp_flush_target = make_write_port<Target>("DECODE_2_FETCH_TARGET", Port::BW);
    wp_bp_update = make_write_port<BPInterface>("DECODE_2_FETCH", Port::BW);
}

template <typename FuncInstr>
auto Decode<FuncInstr>::read_instr( Cycle cycle) const
{
    if ( rp_stall_datapath->is_ready( cycle))
        return std::pair{ rp_stall_datapath->read( cycle), true};

    return std::pair{ rp_datapath->read( cycle), false};
}

template <typename FuncInstr>
bool Decode<FuncInstr>::is_misprediction( const Instr& instr, const BPInterface& bp_data)
{
    if ( ( instr.is_direct_jump() || instr.is_indirect_jump()) && !bp_data.is_taken)
        return true;

    // 'likely' branches, which are not in BTB, are purposely considered as mispredictions
    if ( instr.is_likely_branch() && !bp_data.is_hit)
        return true;

    return ( ( instr.is_direct_jump() || instr.is_branch())
        && bp_data.target != instr.get_decoded_target()
        && bp_data.is_taken);
}

template<typename FuncInstr>
bool Decode<FuncInstr>::is_flush( Cycle cycle) const
{
    return ( rp_flush->is_ready( cycle) && rp_flush->read( cycle))
        || ( rp_flush_fetch->is_ready( cycle) && rp_flush_fetch->read( cycle));
}

template<typename FuncInstr>
void Decode<FuncInstr>::clock( Cycle cycle)
{
    sout << "decode  cycle " << std::dec << cycle << ": ";

    const bool has_trap = rp_trap->is_ready( cycle) && rp_trap->read( cycle);
    const bool has_flush = is_flush( cycle);

    bypassing_unit->update();

    /* trace new instruction if needed */
    if ( rp_bypassing_unit_notify->is_ready( cycle))
    {
        auto instr = rp_bypassing_unit_notify->read( cycle);
        bypassing_unit->trace_new_instr( instr);
    }

    /* update bypassing unit because of misprediction */
    if ( rp_bypassing_unit_flush_notify->is_ready( cycle) || has_trap)
        bypassing_unit->handle_flush();

    if ( has_flush || has_trap)
    {
        sout << "flush\n";
        return;
    }

    /* check if there is something to process */
    if ( !rp_datapath->is_ready( cycle) && !rp_stall_datapath->is_ready( cycle))
    {
        sout << "bubble\n";
        return;
    }

    auto[instr, from_stall] = read_instr( cycle);

    if ( instr.is_jump())
        num_jumps++;

    /* handle misprediction */
    if ( is_misprediction( instr, instr.get_bp_data()))
    {
        num_mispredictions++;

        /* acquiring real information for BPU */
        wp_bp_update->write( instr.get_bp_upd(), cycle);

        // flushing fetch stage, instr fetch will appear at decode stage next clock,
        // so we send flush signal to decode
        if ( !bypassing_unit->is_stall( instr))
            wp_flush_fetch->write( true, cycle);

        /* sending valid PC to fetch stage */
        if ( !from_stall)
        {
            wp_flush_target->write( instr.get_actual_decoded_target(), cycle);
            sout << "\nmisprediction on ";
        }
    }

    if ( bypassing_unit->get_operation_latency() <= 1_lt)
    {
        const auto& register1 = instr.get_src(0);
        const auto& register2 = instr.get_src(1);

        bool is_reg1_zero = register1.is_zero();
        bool is_reg2_zero = register2.is_zero();
        bool is_register1_register2 = register1 == register2;

        int value_1 = 0;
        int value_2 = 1;
        if( (! is_reg1_zero) && (! is_reg2_zero))
        {
            set_registers();
            get_path( value_1, &register_1);
            get_path( value_2, &register_2);
            auto wps_port = wps_command;
            if ( is_register_1 || is_register_2) {
                if ( is_register_1) {
                    alu_number_decoder = register_1.value_alu;
                    instr.set_alu_number( register_1.value_alu);
                } else {
                    alu_number_decoder = register_2.value_alu;
                    instr.set_alu_number( register_2.value_alu);
                }
                auto c_1 = bypassing_unit->get_bypass_command( instr, value_1, register_1.path_number);
                wps_port.at( value_1)->write( c_1, cycle);
                auto c_2 = bypassing_unit->get_bypass_command( instr, value_2, register_2.path_number);
                wps_port.at( value_2)->write( c_2, cycle);
            }

            else if ( !( is_register1_register2))
            {
                if ( register_1.instruction > 1) {
                    alu_number_decoder = register_1.value_alu;
                    instr.set_alu_number( register_1.value_alu);
                    rf->read_source( &instr, value_1);

                } else if ( register_2.instruction > 1) {
                    alu_number_decoder = register_2.value_alu;
                    instr.set_alu_number( register_2.value_alu);
                    rf->read_source( &instr, value_2);
                }
                auto c_1 = bypassing_unit->get_bypass_command( instr, value_1, register_1.path_number);
                wps_port.at( value_1)->write( c_1, cycle);
                auto c_2 = bypassing_unit->get_bypass_command( instr, value_2, register_2.path_number);
                wps_port.at( value_2)->write( c_2, cycle);

            }
            else if ((is_register1_register2)) {
                auto c_1 = bypassing_unit->get_bypass_command( instr, value_1, register_2.path_number);
                wps_port.at( value_1)->write( c_1, cycle);
                auto c_2 = bypassing_unit->get_bypass_command( instr, value_2, register_2.path_number);
                wps_port.at( value_2)->write( c_2, cycle);
            }
        }
        else
        {
           if (! is_register_1)
                rf->read_source(&instr, 0);
            if (! is_register_2)
                rf->read_source(&instr, 1);
            alu_number_decoder = 0;
            instr.set_alu_number(alu_number_decoder);
        }
    }
    else
    {
        // data hazard, stalling pipeline
        wp_stall->write( true, cycle);
        wp_stall_datapath->write( instr, cycle);
        sout << instr << " (data hazard)\n";
        return;
    }

    /* notify bypassing unit about new instruction */
    wp_bypassing_unit_notify->write( instr, cycle);

    /* log */
    sout << instr << std::endl;

    wp_datapath->write( std::move( instr), cycle);
}


template <typename FuncInstr>
void Decode<FuncInstr>::get_path(uint8 reg, struct registers * r)
{
    if ( decode_to_execute.registers[reg])
    {
        r->value_alu = 1;
        r->path_number = 1;
        r->instruction = 2;
    }
    else if ( decode_to_execute.registers[reg] && decode_to_execute.alu_number == 0)
    {
        r->value_alu = 0;
        r->path_number = 1;
        r->instruction = 1;
    }
    else if ( decode_to_execute.registers[reg] && decode_to_execute.alu_number == 1)
    {
        r->value_alu = 1;
        r->path_number = 5;
        r->instruction = 2;
    }
    else if ( execute_to_mem.registers[reg])
    {
        r->value_alu = 0;
        r->path_number = 2;
        r->instruction = 1;
    }
    else if ( execute_to_late_alu.registers[reg])
    {
        r->value_alu = 0;
        r->path_number = 5;
        r->instruction = 1;
    }
    else if ( late_alu_to_writeback.registers[reg] || mem_to_writeback.registers[reg] )
    {
        r->value_alu = 0;
        r->path_number = 3;
        r->instruction = 1;
    }
    else
    {
        r->path_number = -1;
    }
}


template <typename FuncInstr>
void Decode<FuncInstr>::set_registers()
{
    int value_1 = 0;
    int value_2 = 1;

    is_register_1 = false;
    if( decode_to_execute.registers[value_1]
       || execute_to_late_alu.registers[value_1] || execute_to_mem.registers[value_1]
       || late_alu_to_writeback.registers[value_1] || mem_to_writeback.registers[value_1] )
        is_register_1 = true;

    is_register_2 = false;
    if( decode_to_execute.registers[value_2]
       || execute_to_late_alu.registers[value_2] || execute_to_mem.registers[value_2]
       || late_alu_to_writeback.registers[value_2] || mem_to_writeback.registers[value_2])
        is_register_2 = true;
}



#include <mips/mips.h>
#include <risc_v/risc_v.h>

template class Decode<BaseMIPSInstr<uint32>>;
template class Decode<BaseMIPSInstr<uint64>>;
template class Decode<RISCVInstr<uint32>>;
template class Decode<RISCVInstr<uint64>>;
template class Decode<RISCVInstr<uint128>>;

