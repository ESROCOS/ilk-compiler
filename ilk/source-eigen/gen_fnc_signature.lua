local constants = require('ilk.constants')

local generate_output_signatures = function(olist)
    local outputs = {}
    utils.foreach(function(v, k)
        local output_type = v[1]
        local output_name = v[2]
        outputs[#outputs + 1] = output_type.."& "..output_name
    end, olist)
    local output_string = table.concat(outputs,', ')
    return output_string
end



local function gen_fnc_signature(config, args, idx, robot_name)
    local fd = config.fd or io.stdout
    local idx = idx or 0

    local input_type = robot_name..'::'..constants.input_type
    local ending = ' {'
    if args.is_proto then ending = ';' end
    local ostr = generate_output_signatures(args.olist)

    local ok, res = utils.preproc([[
void $(fnc_name)(const mc_$(sname)& mc, const $(inputtype)& input, $(output))$(ends)
]], {
        table = table,
        space = utils.gen_spaces('\t', idx),
        fnc_name = args.fnc_name,
        sname = args.sname,
        inputtype = input_type,
        output = ostr,
        ends = ending
    })
    if not ok then error(res) end
    fd:write(res)
end


return gen_fnc_signature