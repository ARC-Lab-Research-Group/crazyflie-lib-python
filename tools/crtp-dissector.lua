-- declare our protocol
crtp = Proto("CrazyRealTimeTProtocol","Crazy Real Time Protocol")

-- General CRTP packet fields
local f_crtp_port = ProtoField.uint8("crtp.port", "Port")
local f_crtp_channel = ProtoField.uint8("crtp.channel", "Channel")
local f_crtp_size = ProtoField.uint8("crtp.size", "Size")
local f_crtp_undecoded = ProtoField.string("crtp.undecoded", "Undecoded")

-- Specialized CRTP service fields
local f_crtp_console_text = ProtoField.string("crtp.console_text", "Text", base.ASCII)
local f_crtp_parameter_varid = ProtoField.uint16("crtp.parameter_varid", "Variable Id")

local f_crtp_parameter_val_uint = ProtoField.uint32("crtp.parameter_val_uint", "Value uint")
local f_crtp_parameter_val_int = ProtoField.int32("crtp.parameter_val_int", "Value int")
local f_crtp_parameter_val_float = ProtoField.float("crtp.parameter_val_float", "Value float")

local f_crtp_setpoint_hl_command = ProtoField.string("crtp.setpoint_hl_command", "Command")
local f_crtp_setpoint_hl_retval = ProtoField.uint8("crtp.setpoint_hl_retval", "Return Value")

local f_crtp_setpoint_hl_groupmask = ProtoField.uint8("crtp.setpoint_hl_groupmask", "Group Mask")
local f_crtp_setpoint_hl_id = ProtoField.uint8("crtp.setpoint_hl_id", "Trajectory Id")

local f_crtp_setpoint_hl_height = ProtoField.float("crtp.setpoint_hl_height", "Height")
local f_crtp_setpoint_hl_yaw = ProtoField.float("crtp.setpoint_hl_yaw", "Yaw")
local f_crtp_setpoint_hl_use_yaw = ProtoField.bool("crtp.setpoint_hl_use_yaw", "Use Current Yaw")
local f_crtp_setpoint_hl_relative = ProtoField.bool("crtp.setpoint_hl_relative", "Relative")
local f_crtp_setpoint_hl_duration = ProtoField.float("crtp.setpoint_hl_duration", "Duration")
local f_crtp_setpoint_hl_timescale = ProtoField.float("crtp.setpoint_hl_timescale", "Timescale")

local f_crtp_setpoint_hl_x = ProtoField.float("crtp.setpoint_hl_x", "X")
local f_crtp_setpoint_hl_y = ProtoField.float("crtp.setpoint_hl_y", "Y")
local f_crtp_setpoint_hl_z = ProtoField.float("crtp.setpoint_hl_z", "Z")


-- All possible fields registred
crtp.fields = {
    f_crtp_port, f_crtp_channel, f_crtp_console_text, f_crtp_size,
    f_crtp_parameter_varid, f_crtp_parameter_val_uint,
    f_crtp_parameter_val_int, f_crtp_parameter_val_float,
    f_crtp_setpoint_hl_command, f_crtp_setpoint_hl_retval,
    f_crtp_setpoint_hl_use_yaw, f_crtp_setpoint_hl_yaw,
    f_crtp_setpoint_hl_groupmask, f_crtp_setpoint_hl_duration,
    f_crtp_setpoint_hl_height, f_crtp_setpoint_hl_relative,
    f_crtp_setpoint_hl_x, f_crtp_setpoint_hl_y, f_crtp_setpoint_hl_z,
    f_crtp_setpoint_hl_id, f_crtp_setpoint_hl_timescale, f_crtp_undecoded
}

local Links = {
    UNKNOWN = 0,
    RADIO = 1,
    USB = 2,
}

local link = Links.UNKNOWN
local undecoded = 0
local crtp_start = 0

-- Analye port and channel and figure what service (port name / channel name)
-- we are dealing with

local Ports = {
    Console = 0x0,
    Parameters = 0x2,
    Commander = 0x3,
    Memory = 0x4,
    Logging = 0x5,
    Localization = 0x6,
    Commander_Generic = 0x7,
    Setpoint_Highlevel = 0x8,
    Platform = 0xD,
    All = 0xF
}

function get_crtp_port_channel_names(port, channel)
    local port_name = "Unknown"
    local channel_name = nil

    if port == Ports.Console and channel == 0 then port_name = "Console"

    elseif port == 0x02 then
        port_name = "Parameters"
        if channel == 0 then channel_name = "Table Of Contents"
        elseif channel == 1 then channel_name = "Read"
        elseif channel == 2 then channel_name = "Write"
        elseif channel == 3 then channel_name = "Misc"
        end

    elseif port == 0x03 then port_name = "Commander"

    elseif port == 0x04 then
        port_name = "Memory"
        if channel == 0 then channel_name = "Information"
        elseif channel == 1 then channel_name = "Read"
        elseif channel == 2 then channel_name = "Write"
        end

    elseif port == 0x05 then
        port_name = "Logging"
        if channel == 0 then channel_name = "Table Of Contents"
        elseif channel == 1 then channel_name = "Settings"
        elseif channel == 2 then channel_name = "Log data"
        end

    elseif port == 0x06 then port_name = "Localization"
        if channel == 0 then channel_name = "Position"
        elseif channel == 1 then channel_name = "Generic"
        end

    elseif port == 0x07 then port_name = "Commander Generic"
    elseif port == 0x08 then port_name = "Setpoint Highlevel"
    elseif port == 0x0D then
        port_name = "Platform"
        if channel == 0 then channel_name = "Platform Command"
        elseif channel == 1 then channel_name = "Version Command"
        elseif channel == 2 then channel_name = "App Layer"
        end

    elseif port == 0x0F then
        port_name = "Link Control"
        if channel == 1 then channel_name = "Link Service Source"
        end

    elseif port == 0xFF then port_name = "ALL" end

    return port_name, channel_name
end

function format_address(buffer)
    if link == Links.RADIO then
        addr = buffer(2, 5):bytes():tohex()
        port = buffer(7, 1):uint()

        addr = addr:gsub("..", ":%0"):sub(2)
        port = tostring(port)

        return addr .. " (" .. port .. ")"
    elseif link == Links.USB then
        return buffer(2, 12):bytes():tohex()
    end
end

function format_device(buffer)
    if link == Links.RADIO then
        devid = buffer(8, 1):uint()
        return "Radio #" .. tostring(devid)
    elseif link == Links.USB then
        devid = buffer(15, 1):uint()
        return "USB #" .. tostring(devid)
    end

end

function handle_setpoint_highlevel(tree, receive, buffer, channel, size)
    local Commands = {
        COMMAND_SET_GROUP_MASK          = 0,
        COMMAND_TAKEOFF                 = 1,
        COMMAND_LAND                    = 2,
        COMMAND_STOP                    = 3,
        COMMAND_GO_TO                   = 4,
        COMMAND_START_TRAJECTORY        = 5,
        COMMAND_DEFINE_TRAJECTORY       = 6,
        COMMAND_TAKEOFF_2               = 7,
        COMMAND_LAND_2                  = 8,
        COMMAND_TAKEOFF_WITH_VELOCITY   = 9,
        COMMAND_LAND_WITH_VELOCITY      = 10,
    }

    local height = nil
    local duration = nil
    local yaw = nil
    local group_mask = nil
    local use_yaw = nil
    local relative = nil
    local x = nil
    local y = nil
    local z = nil
    local id = nil
    local timescale = nil

    local port_tree = tree:add(crtp, port_name)

    local cmd = buffer(crtp_start + 1, 1):uint()
    local cmd_str = "Unknown"

    if cmd == Commands.COMMAND_SET_GROUP_MASK then
        cmd_str = "Set Group Mask"

        -- struct data_set_group_mask {
        --    uint8_t groupMask;        // mask for which CFs this should apply to
        --  } __attribute__((packed));
        if receive == 0 then
            group_mask = buffer(11, 1):uint()
        end
    elseif cmd == Commands.COMMAND_TAKEOFF then cmd_str = "Take Off (deprecated)"
    elseif cmd == Commands.COMMAND_LAND then cmd_str = "Land (deprecated)"
    elseif cmd == Commands.COMMAND_STOP then
        cmd_str = "Stop"

        -- struct data_stop {
        --    uint8_t groupMask;        // mask for which CFs this should apply to
        --  } __attribute__((packed));
        if receive == 0 then
            group_mask = buffer(crtp_start + 2, 1):uint()
        end
        undecoded = undecoded - 1
    elseif cmd == Commands.COMMAND_GO_TO then
        cmd_str = "Go To"

        -- struct data_go_to {
        --    uint8_t groupMask; // mask for which CFs this should apply to
        --    uint8_t relative;  // set to true, if position/yaw are relative to current setpoint
        --    float x; // m
        --    float y; // m
        --    float z; // m
        --    float yaw; // rad
        --    float duration; // sec
        --  } __attribute__((packed));
        if receive == 0 then
            group_mask = buffer(crtp_start + 2, 1):uint()
            relative = buffer(crtp_start + 3, 1):uint()
            x = buffer(crtp_start + 4, 4):le_float()
            y = buffer(crtp_start + 8, 4):le_float()
            z = buffer(crtp_start + 12, 4):le_float()
            yaw = buffer(crtp_start + 16, 4):le_float()
            duration = buffer(crtp_start + 20, 4):le_float()
            undecoded = undecoded - 24

        end
    elseif cmd == Commands.COMMAND_START_TRAJECTORY then
        cmd_str = "Start Trajectory"

        -- struct data_start_trajectory {
        --   uint8_t groupMask; // mask for which CFs this should apply to
        --   uint8_t relative;  // set to true, if trajectory should be shifted to current setpoint
        --   uint8_t reversed;  // set to true, if trajectory should be executed in reverse
        --   uint8_t trajectoryId; // id of the trajectory (previously defined by COMMAND_DEFINE_TRAJECTORY)
        --   float timescale; // time factor; 1 = original speed; >1: slower; <1: faster
        --  } __attribute__((packed));
        if receive == 0 then
            group_mask = buffer(crtp_start + 2, 1):uint()
            relative = buffer(crtp_start + 3, 1):uint()
            reversed = buffer(crtp_start + 4, 1):uint()
            id = buffer(crtp_start + 5, 1):uint()
            timescale = buffer(crtp_start + 6):float()
            undecoded = undecoded - 10
        end
    elseif cmd == Commands.COMMAND_DEFINE_TRAJECTORY then
        cmd_str = "Define Trajectory"

        -- struct data_define_trajectory {
        --    uint8_t trajectoryId;
        --    struct trajectoryDescription description;
        --  } __attribute__((packed));
        if receive == 0 then
            id = buffer(crtp_start + 2, 1):uint()
            undecoded = undecoded - 1
        end
    elseif cmd == Commands.COMMAND_TAKEOFF_2 then
        cmd_str = "Take Off"

        -- struct data_takeoff_2 {
        -- uint8_t groupMask;        // mask for which CFs this should apply to
        -- float height;             // m (absolute)
        -- float yaw;                // rad
        -- bool useCurrentYaw;       // If true, use the current yaw (ignore the yaw parameter)
        -- float duration;           // s (time it should take until target height is reached)
        -- } __attribute__((packed));
        if receive == 0 then
            group_mask = buffer(crtp_start + 2, 1):uint()
            height = buffer(crtp_start + 3, 4):le_float()
            yaw =  buffer(crtp_start + 7, 4):le_float()
            use_yaw = buffer(crtp_start + 11, 1):uint()
            duration = buffer(crtp_start + 12, 4):le_float()
            undecoded = undecoded - 16
        end

    elseif cmd == Commands.COMMAND_LAND_2 then
        cmd_str = "Land"
        -- struct data_land_2 {
        -- uint8_t groupMask;        // mask for which CFs this should apply to
        -- float height;             // m (absolute)
        -- float yaw;                // rad
        -- bool useCurrentYaw;       // If true, use the current yaw (ignore the yaw parameter)
        -- float duration;           // s (time it should take until target height is reached)
        -- } __attribute__((packed));
        if receive == 0 then
            group_mask = buffer(crtp_start + 2, 1):uint()
            height = buffer(crtp_start + 3, 4):le_float()
            yaw =  buffer(crtp_start + 7, 4):le_float()
            use_yaw = buffer(crtp_start + 11, 1):uint()
            duration = buffer(crtp_start + 12, 4):le_float()
            undecoded = undecoded - 16
        end

    elseif cmd == Commands.COMMAND_TAKEOFF_WITH_VELOCITY then cmd_str = "Take Off With Velocity"
    elseif cmd == Commands.COMMAND_LAND_WITH_VELOCITY then cmd_str = "Land With Velocity" end

    port_tree:add_le(f_crtp_setpoint_hl_command, cmd_str)
    local success = true
    if receive == 1 then
        retval = buffer(crtp_start + 4):uint()
        local success = (retval == 0) and "Success" or "Failure"
        port_tree:add_le(f_crtp_setpoint_hl_retval, retval):append_text(" (" .. success .. ")")
        undecoded = undecoded - 4
    end

    if id then
        port_tree:add_le(f_crtp_setpoint_hl_id, id)
    end
    if timescale then
        port_tree:add_le(f_crtp_setpoint_hl_timescale, timescale)
    end
    if group_mask then
        port_tree:add_le(f_crtp_setpoint_hl_groupmask, group_mask)
    end
    if relative then
        port_tree:add_le(f_crtp_setpoint_hl_relative, relative)
    end
    if height then
        port_tree:add_le(f_crtp_setpoint_hl_height, height):append_text(" (m)")
    end
    if x then
        port_tree:add_le(f_crtp_setpoint_hl_x, x)
    end
    if y then
        port_tree:add_le(f_crtp_setpoint_hl_y, y)
    end
    if z then
        port_tree:add_le(f_crtp_setpoint_hl_z, z)
    end
    if yaw then
        port_tree:add_le(f_crtp_setpoint_hl_yaw, yaw)
    end
    if use_yaw then
        port_tree:add_le(f_crtp_setpoint_hl_use_yaw, use_yaw)
    end
    if duration then
        port_tree:add_le(f_crtp_setpoint_hl_duration, duration):append_text(" (s)")
    end
end

function handle_parameter_port(tree, buffer, channel, size)
     -- Read or Write
     if (channel == 1 or channel == 2) and size > 2 then

        -- Add variable id
        local var_id = buffer(crtp_start + 1, 2):le_uint()
        local port_tree = tree:add(crtp, port_name)
        port_tree:add_le(f_crtp_parameter_varid, var_id)

        -- Add value
        if size > 3 then
            port_tree:add_le(f_crtp_parameter_val_uint, buffer(crtp_start + 4):le_uint())
            port_tree:add_le(f_crtp_parameter_val_int, buffer(crtp_start + 4):le_int())
        end
        if size >= 7 then
            port_tree:add_le(f_crtp_parameter_val_float, buffer(crtp_start + 4):le_float())
        end
        undecoded = 0
    end
end

-- create a function to dissect it, layout:
-- | link_type | receive| address       | channel | radio devid | crtp header | crtp data |
-- | 1 byte    | 1 byte | 5 or 12 bytes |  1 byte |    1 byte   |    1 byte   |   n bytes |
function crtp.dissector(buffer, pinfo, tree)
    pinfo.cols.protocol = "CRTP"

    link = buffer(0, 1):uint()
    if link == Links.RADIO then
        crtp_start = 9
    elseif link == Links.USB then
        crtp_start = 16
    end

    if buffer:len() <= crtp_start then return end

    local receive = buffer(1, 1):uint()
    if receive == 1 then
        pinfo.cols.dst = format_device(buffer)
        pinfo.cols.src = format_address(buffer)
    else
        pinfo.cols.dst = format_address(buffer)
        pinfo.cols.src = format_device(buffer)
    end

    local subtree = tree:add(crtp, "CRTP Packet")
    local header = bit.band(buffer(crtp_start, 1):uint(), 0xF3)
    local crtp_port = bit.rshift(bit.band(header, 0xF0), 4)
    local crtp_channel = bit.band(header, 0x03)

    -- Add CRTP packet size:
    -- receive_byte + address + channel + devid = 8
    -- Rest is CRTP packet
    local crtp_size = buffer:len() - crtp_start
    subtree:add_le(f_crtp_size, crtp_size)

    undecoded = crtp_size - 1

    -- Check for safelink packet
    if crtp_size == 3 and header == 0xF3 and buffer(crtp_start + 1, 1):uint() == 0x05 then
        pinfo.cols.info = "SafeLink"
        return
    end

    -- Get port and channel name
    port_name, channel_name = get_crtp_port_channel_names(crtp_port, crtp_channel)

    -- Display port in info column
    pinfo.cols.info = port_name

    -- Add to CRTP tree
    subtree:add_le(f_crtp_port, crtp_port):append_text(" (" .. port_name .. ")")
    if channel_name then
        subtree:add_le(f_crtp_channel, crtp_channel):append_text(" (" .. channel_name .. ")")
    else
        subtree:add_le(f_crtp_channel, crtp_channel)
    end

    -- Check for special handling

    -- Console, we can add text
    if crtp_port == Ports.Console then
        local port_tree = tree:add(crtp, port_name)
        port_tree:add_le(f_crtp_console_text, buffer(crtp_start + 1):string())
        undecoded = 0
    end

    if crtp_port == Ports.Parameters then
        handle_parameter_port(tree, buffer, crtp_channel, crtp_size)
    end

    if crtp_port == Ports.Setpoint_Highlevel then
        handle_setpoint_highlevel(tree, receive, buffer, crtp_channel, crtp_size)
    end

    if undecoded > 0 then
        local from = crtp_start + (crtp_size - undecoded)
        subtree:add_le(f_crtp_undecoded, buffer(from, undecoded):bytes():tohex())
    end
end

wtap_encap = DissectorTable.get("wtap_encap")
wtap_encap:add(wtap.USER15, crtp)
