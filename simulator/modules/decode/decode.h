/**
 * decode.h - simulation of decode stage
 * Copyright 2015-2018 MIPT-MIPS
 */


#ifndef DECODE_H
#define DECODE_H

#include "bypass/data_bypass.h"

#include <func_sim/rf/rf.h>
#include <modules/core/perf_instr.h>
#include <modules/ports_instance.h>

template <typename FuncInstr>
class Decode : public Module
{
    using Register = typename FuncInstr::Register;
    using Instr = PerfInstr<FuncInstr>;
    using BypassingUnit = DataBypass<FuncInstr>;
    static constexpr const uint8 SRC_REGISTERS_NUM = 2;

public:
    explicit Decode( Module* parent);
    void clock( Cycle cycle);
    void set_RF( RF<FuncInstr>* value) { rf = value;}
    void set_wb_bandwidth( uint32 wb_bandwidth) { bypassing_unit->set_bandwidth( wb_bandwidth);}
    auto get_mispredictions_num() const { return num_mispredictions; }
    auto get_jumps_num() const { return num_jumps; }

private:
    auto read_instr( Cycle cycle) const;
    bool is_flush( Cycle cycle) const;
    bool is_register_1 = false;
    bool is_register_2 = false;
    static bool is_misprediction( const Instr& instr, const BPInterface& bp_data);

    uint64 num_jumps          = 0;
    uint64 num_mispredictions = 0;
    int alu_number_decoder   = 0;
    RF<FuncInstr>* rf = nullptr;
    std::unique_ptr<BypassingUnit> bypassing_unit = nullptr;

    /* Inputs */
    ReadPort<Instr>* rp_datapath = nullptr;
    ReadPort<Instr>* rp_stall_datapath = nullptr;
    ReadPort<bool>* rp_flush = nullptr;
    ReadPort<Instr>* rp_bypassing_unit_notify = nullptr;
    ReadPort<bool>* rp_bypassing_unit_flush_notify = nullptr;
    ReadPort<bool>* rp_flush_fetch = nullptr;
    ReadPort<bool>* rp_trap = nullptr;

    /* Outputs */
    WritePort<Instr>* wp_datapath = nullptr;
    WritePort<Instr>* wp_stall_datapath = nullptr;
    WritePort<bool>* wp_stall = nullptr;
    WritePort<Instr>* wp_bypassing_unit_notify = nullptr;
    WritePort<BPInterface>* wp_bp_update = nullptr;
    std::array<WritePort<BypassCommand<Register>>*, SRC_REGISTERS_NUM> wps_command;
    std::array<WritePort<BypassCommand<Register>>*, SRC_REGISTERS_NUM> wps_command_late_alu;
    WritePort<bool>* wp_flush_fetch = nullptr;
    WritePort<Target>* wp_flush_target = nullptr;

    struct registers
    {
        int8 path_number = 0;
        uint8 value_alu = 0;
        uint8 instruction = 0;
    } register_1, register_2;

    struct path

        bool registers[2] = {false, false};
        uint8 alu_number = 0;
    } decode_to_execute, execute_to_late_alu, execute_to_mem, late_alu_to_writeback, mem_to_writeback;

    void get_path(uint8 reg, struct registers * r);
    void set_registers();
};


#endif // DECODE_H
