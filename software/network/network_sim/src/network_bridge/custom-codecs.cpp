#include "custom-codecs.h"

#include <iostream>

std::map<std::string, bot_procman::orders_t> PMDOrdersCodec::state_;
std::map<std::string, bot_procman::info_t> PMDInfoCodec::state_;


int DRCEmptyIdentifierCodec::currently_decoded_id = 0;


using goby::glog;
using namespace goby::common::logger;




goby::acomms::Bitset DRCPresenceBitStringCodec::encode()
{
    return goby::acomms::Bitset(min_size(), 0);
}


goby::acomms::Bitset DRCPresenceBitStringCodec::encode(const std::string& wire_value)
{
    std::string s = wire_value;
    if(s.size() > dccl_field_options().max_length())
    {
        goby::glog.is(DEBUG2) && goby::glog << group(goby::acomms::DCCLCodec::glog_encode_group()) << warn << "String " << s <<  " exceeds `dccl.max_length`, truncating" << std::endl;
        s.resize(dccl_field_options().max_length()); 
    }

    goby::acomms::Bitset bits;
    bits.from_byte_string(s);
    bits.resize(size(s));
    
    bits <<= 1;
    bits.set(0, true); // presence bit    
    
    return bits;
}

unsigned DRCPresenceBitStringCodec::size()
{
    return min_size();    
}


unsigned DRCPresenceBitStringCodec::size(const std::string& wire_value)
{
    return (wire_value.size() + 1) * goby::acomms::BITS_IN_BYTE + min_size();
}


std::string DRCPresenceBitStringCodec::decode(goby::acomms::Bitset* bits)
{
    if(bits->to_ulong())
    {
        // grabs more bits to add to the MSBs of `bits`
        bits->get_more_bits(goby::acomms::BITS_IN_BYTE);
        *bits >>= 1;
        bits->resize(goby::acomms::BITS_IN_BYTE);

        std::string out;
        while(bits->to_ulong())
        {
            out.push_back(bits->to_byte_string()[0]);
            bits->get_more_bits(goby::acomms::BITS_IN_BYTE);
            *bits >>= goby::acomms::BITS_IN_BYTE;
            bits->resize(goby::acomms::BITS_IN_BYTE);
        }
        return out;
    }
    else
    {
        throw(goby::acomms::DCCLNullValueException());
    }
}

unsigned DRCPresenceBitStringCodec::max_size()
{
    return dccl_field_options().max_length() * goby::acomms::BITS_IN_BYTE + min_size();
}

unsigned DRCPresenceBitStringCodec::min_size()
{
    return 1; // presence bit
}

void DRCPresenceBitStringCodec::validate()
{
    require(dccl_field_options().has_max_length(), "missing (goby.field).dccl.max_length");
}


// PMD_ORDERS

bool PMDOrdersCodec::make_diff(const bot_procman::orders_t& orders, const bot_procman::orders_t& reference, drc::PMDOrdersDiff* diff)
{

    // modulus to tenth of seconds, so full messages can be resolved to ~25 seconds
    diff->set_reference_time((reference.utime / 100000) % 256);
    
    diff->set_utime(orders.utime - reference.utime);

    if(orders.sheriff_name != reference.sheriff_name)
        diff->set_sheriff_name(orders.sheriff_name);    

    for(int i = 0, n = orders.cmds.size(); i < n; ++i)
    {
        const bot_procman::sheriff_cmd_t& cmd = orders.cmds[i];

        drc::PMDOrdersDiff::PMDSheriffCmdDiff* diff_cmd = diff->add_cmds();
        
        bool has_ref_cmd = i < reference.cmds.size();
        
        const bot_procman::sheriff_cmd_t* ref_cmd = has_ref_cmd ? &reference.cmds[i] : 0;
        
        if(!has_ref_cmd || cmd.name != ref_cmd->name)
            diff_cmd->set_name(cmd.name);
        if(!has_ref_cmd || cmd.nickname != ref_cmd->nickname)
            diff_cmd->set_nickname(cmd.nickname);
        if(!has_ref_cmd || cmd.group != ref_cmd->group)
            diff_cmd->set_group(cmd.group);
        if(!has_ref_cmd || cmd.desired_runid != ref_cmd->desired_runid)
            diff_cmd->set_desired_runid(cmd.desired_runid);
        if(!has_ref_cmd || cmd.force_quit != ref_cmd->force_quit)
            diff_cmd->set_force_quit(cmd.force_quit);
        if(!has_ref_cmd || cmd.sheriff_id != ref_cmd->sheriff_id)
            diff_cmd->set_sheriff_id(cmd.sheriff_id);

        // set this to work around https://bugs.launchpad.net/dccl/+bug/1177415
        diff_cmd->set_auto_respawn(cmd.auto_respawn);
    }
            
    glog.is(VERBOSE) && glog << "Made PMD_ORDERS diff: " << diff->ShortDebugString() << std::endl;
    return true;
}

bool PMDOrdersCodec::reverse_diff(bot_procman::orders_t* orders, const bot_procman::orders_t& reference, const drc::PMDOrdersDiff& diff)
{
    glog.is(VERBOSE) && glog << "Received PMD_ORDERS diff: " << diff.ShortDebugString() << std::endl;

    
    if(((reference.utime / 100000) % 256) != diff.reference_time())
        {
            glog.is(WARN) && glog << "Wrong time reference, cannot reassemble diff. " << std::endl;
            return false;
        }

    orders->utime = reference.utime + diff.utime();
    orders->host = reference.host;
    orders->sheriff_name = diff.has_sheriff_name() ? diff.sheriff_name() : reference.sheriff_name;
    orders->ncmds = diff.cmds_size();
    orders->nvars = 0;

    for(int i = 0, n = diff.cmds_size(); i < n; ++i)
    {
        bot_procman::sheriff_cmd_t cmd;
        const drc::PMDOrdersDiff::PMDSheriffCmdDiff& diff_cmd = diff.cmds(i);
        
        bool has_ref_cmd = i < reference.cmds.size();
        
        const bot_procman::sheriff_cmd_t* ref_cmd = has_ref_cmd ? &reference.cmds[i] : 0;

        cmd.name = (!has_ref_cmd || diff_cmd.has_name()) ? diff_cmd.name() : ref_cmd->name;
        cmd.nickname = (!has_ref_cmd || diff_cmd.has_nickname()) ? diff_cmd.nickname() : ref_cmd->nickname;
        cmd.group = (!has_ref_cmd || diff_cmd.has_group()) ? diff_cmd.group() : ref_cmd->group;
        cmd.desired_runid = (!has_ref_cmd || diff_cmd.has_desired_runid()) ? diff_cmd.desired_runid() : ref_cmd->desired_runid;
        cmd.force_quit = (!has_ref_cmd || diff_cmd.has_force_quit()) ? diff_cmd.force_quit() : ref_cmd->force_quit;
        cmd.sheriff_id = (!has_ref_cmd || diff_cmd.has_sheriff_id()) ? diff_cmd.sheriff_id() : ref_cmd->sheriff_id;
        cmd.auto_respawn = (!has_ref_cmd || diff_cmd.has_auto_respawn()) ? diff_cmd.auto_respawn() : ref_cmd->auto_respawn;
        
        orders->cmds.push_back(cmd);
    }
    
        
    return true;
}

// PMD_INFO

bool PMDInfoCodec::make_diff(const bot_procman::info_t& info, const bot_procman::info_t& reference, drc::PMDInfoDiff* diff)
{
    diff->set_reference_time((reference.utime / 100000) % 256);

    diff->set_utime(info.utime - reference.utime);

    if(info.cpu_load != reference.cpu_load)
        diff->set_cpu_load(info.cpu_load);
    if(info.phys_mem_total_bytes != reference.phys_mem_total_bytes)
        diff->set_phys_mem_total_kbytes(info.phys_mem_total_bytes / 1024);
    if(info.phys_mem_free_bytes != reference.phys_mem_free_bytes)
        diff->set_phys_mem_free_kbytes(info.phys_mem_free_bytes / 1024);
    if(info.swap_total_bytes != reference.swap_total_bytes)
        diff->set_swap_total_kbytes(info.swap_total_bytes / 1024);
    if(info.swap_free_bytes != reference.swap_free_bytes)
        diff->set_swap_free_kbytes(info.swap_free_bytes / 1024);

    for(int i = 0, n = info.cmds.size(); i < n; ++i)
    {
        const bot_procman::deputy_cmd_t& cmd = info.cmds[i];

        drc::PMDInfoDiff::PMDDeputyCmdDiff* diff_cmd = diff->add_cmds();
        
        bool has_ref_cmd = i < reference.cmds.size();
        
        const bot_procman::deputy_cmd_t* ref_cmd = has_ref_cmd ? &reference.cmds[i] : 0;
        
        if(!has_ref_cmd || cmd.name != ref_cmd->name)
            diff_cmd->set_name(cmd.name);
        if(!has_ref_cmd || cmd.nickname != ref_cmd->nickname)
            diff_cmd->set_nickname(cmd.nickname);
        if(!has_ref_cmd || cmd.group != ref_cmd->group)
            diff_cmd->set_group(cmd.group);
        if(!has_ref_cmd || cmd.pid != ref_cmd->pid)
            diff_cmd->set_pid(cmd.pid);
        if(!has_ref_cmd || cmd.actual_runid != ref_cmd->actual_runid)
            diff_cmd->set_actual_runid(cmd.actual_runid);
        if(!has_ref_cmd || cmd.exit_code != ref_cmd->exit_code)
            diff_cmd->set_exit_code(cmd.exit_code);
        if(!has_ref_cmd || cmd.cpu_usage != ref_cmd->cpu_usage)
            diff_cmd->set_cpu_usage(cmd.cpu_usage);
        if(!has_ref_cmd || cmd.mem_vsize_bytes != ref_cmd->mem_vsize_bytes)
            diff_cmd->set_mem_vsize_kbytes(cmd.mem_vsize_bytes / 1024);
        if(!has_ref_cmd || cmd.mem_rss_bytes != ref_cmd->mem_rss_bytes)
            diff_cmd->set_mem_rss_kbytes(cmd.mem_rss_bytes / 1024);
        if(!has_ref_cmd || cmd.sheriff_id != ref_cmd->sheriff_id)
            diff_cmd->set_sheriff_id(cmd.sheriff_id);

        // set this to work around https://bugs.launchpad.net/dccl/+bug/1177415
        diff_cmd->set_auto_respawn(cmd.auto_respawn);

    }       
    glog.is(VERBOSE) && glog << "Made PMD_INFO diff: " << diff->ShortDebugString() << std::endl;

    return true;
}

bool PMDInfoCodec::reverse_diff(bot_procman::info_t* info, const bot_procman::info_t& reference, const drc::PMDInfoDiff& diff)
{
    if(((reference.utime / 100000) % 256) != diff.reference_time())
        return false;
    
    info->utime = reference.utime + diff.utime();
    info->host = reference.host;
    
    info->cpu_load = diff.has_cpu_load() ? diff.cpu_load() : reference.cpu_load;
    info->phys_mem_total_bytes = diff.has_phys_mem_total_kbytes() ? diff.phys_mem_total_kbytes() * 1024 : reference.phys_mem_total_bytes;
    info->phys_mem_free_bytes = diff.has_phys_mem_free_kbytes() ? diff.phys_mem_free_kbytes() * 1024 : reference.phys_mem_free_bytes;
    info->swap_total_bytes = diff.has_swap_total_kbytes() ? diff.swap_total_kbytes() * 1024 : reference.swap_total_bytes; 
    info->swap_free_bytes = diff.has_swap_free_kbytes() ? diff.swap_free_kbytes() *  1024 : reference.swap_free_bytes;
    
    info->ncmds = diff.cmds_size();
    
    for(int i = 0, n = diff.cmds_size(); i < n; ++i)
    {
        bot_procman::deputy_cmd_t cmd;
        const drc::PMDInfoDiff::PMDDeputyCmdDiff& diff_cmd = diff.cmds(i);
        
        bool has_ref_cmd = i < reference.cmds.size();
        
        const bot_procman::deputy_cmd_t* ref_cmd = has_ref_cmd ? &reference.cmds[i] : 0;

        cmd.name = (!has_ref_cmd || diff_cmd.has_name()) ? diff_cmd.name() : ref_cmd->name;
        cmd.nickname = (!has_ref_cmd || diff_cmd.has_nickname()) ? diff_cmd.nickname() : ref_cmd->nickname;
        cmd.group = (!has_ref_cmd || diff_cmd.has_group()) ? diff_cmd.group() : ref_cmd->group;
        cmd.pid = (!has_ref_cmd || diff_cmd.has_pid()) ? diff_cmd.pid() : ref_cmd->pid;
        cmd.actual_runid = (!has_ref_cmd || diff_cmd.has_actual_runid()) ? diff_cmd.actual_runid() : ref_cmd->actual_runid;
        cmd.exit_code = (!has_ref_cmd || diff_cmd.has_exit_code()) ? diff_cmd.exit_code() : ref_cmd->exit_code;
        cmd.mem_vsize_bytes = (!has_ref_cmd || diff_cmd.has_mem_vsize_kbytes()) ? diff_cmd.mem_vsize_kbytes() * 1024 : ref_cmd->mem_vsize_bytes;
        cmd.mem_rss_bytes = (!has_ref_cmd || diff_cmd.has_mem_rss_kbytes()) ? diff_cmd.mem_rss_kbytes() * 1024 : ref_cmd->mem_rss_bytes;
        cmd.sheriff_id = (!has_ref_cmd || diff_cmd.has_sheriff_id()) ? diff_cmd.sheriff_id() : ref_cmd->sheriff_id;
        cmd.auto_respawn = (!has_ref_cmd || diff_cmd.has_auto_respawn()) ? diff_cmd.auto_respawn() : ref_cmd->auto_respawn;

        
        info->cmds.push_back(cmd);
    }

    return true;
}


    
