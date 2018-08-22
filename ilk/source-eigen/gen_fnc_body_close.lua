local gen_fnc_body_close = function(config)
    local fd = config.fd or io.stdout
    local ok, res = utils.preproc([[

}
]], {
        table = table
    })
    if not ok then error(res) end
    fd:write(res)
end


return gen_fnc_body_close