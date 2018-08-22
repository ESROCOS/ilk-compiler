local M = {}

local extract_identifiers_in_order = function(model_values)
    local keys_in_order = {}
    for i, id in pairs(model_values) do
        keys_in_order[#keys_in_order + 1] = i
    end
    table.sort(keys_in_order)
    return keys_in_order
end

M.extract_identifiers_in_order = extract_identifiers_in_order
return M
