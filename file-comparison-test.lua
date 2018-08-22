local function default_actual_folder_name()
    return "out"
end

local function default_expected_folder_name()
    return "out_correct"
end

local function default_input_folder_name()
    return "etc/example"
end

local function default_test_name()
    return "Main test"
end

local function default_files()
    local files_default = {}
    files_default["ur5.cpp"] = { "" }
    files_default["ur5.h"] = { "" }
    files_default["ur5_test.cpp"] = { "" }
    files_default["makefile"] = { "" }
    files_default["robot-defs.h"] = { "" }
    return files_default
end

local function read_file_content(path, filename)
    local file = io.open(path .. filename, 'r')
    local data = file:read("*all")
    file:close()
    return data
end

local function preprocess(line_text)
    line_text = line_text:match("^%s*(.-)%s*$")
    return line_text
end


local function compare_files_content(path, filename, expected_folder_name, actual_folder_name)
    local expected_file = io.open(expected_folder_name .. "/" .. path .. filename, 'r')
    local actual_file = io.open(actual_folder_name .. "/" .. path .. filename, 'r')
    local expected_line = expected_file:read()
    local actual_line = actual_file:read()
    local line = 1
    local mismatch = false
    repeat
        local actual_line_preprocessed = preprocess(actual_line)
        local expected_line_preprocessed = preprocess(expected_line)
        if not string.match(actual_line_preprocessed, "(UTC)") and not string.match(expected_line_preprocessed, "(UTC)") then
            if actual_line_preprocessed ~= expected_line_preprocessed then
                mismatch = true
                break
            end
        end
        expected_line = expected_file:read()
        actual_line = actual_file:read()
        line = line + 1
    until expected_line == nil or actual_line == nil

    expected_file:close()
    actual_file:close()
    return mismatch, line, actual_line, expected_line
end

local function read_file_contents(path, filename, expected_folder_name, actual_folder_name)
    local expected = read_file_content(expected_folder_name .. "/" .. path, filename)
    local actual = read_file_content(actual_folder_name .. "/" .. path, filename)
    return actual, expected
end

local function test_file(path, filename)
    local actual, expected = read_file_contents(path, filename)
    return actual == expected
end

local function file_exists(path, filename, actual_folder_name)
    local file = io.open(actual_folder_name .. "/" .. path .. filename, 'r')
    if file == nil then return false end
    file:close()
    return true
end





-- selects the generator from the backend
local function create_file_comparison_test(name, input_folder, expected_folder, actual_folder, files)
    local test_name = name or default_test_name()
    local files_to_compare = files or default_files()
    local expected_folder_name = expected_folder or default_expected_folder_name()
    local actual_folder_name = actual_folder or default_actual_folder_name()
    local input_folder_name = input_folder or default_input_folder_name()

    return function()
        local passed = true
        os.execute('rm -rf out')
        os.execute('lua ilk-compiler.lua -b eigen --indir ' .. input_folder_name .. ' --outdir out --silent')

        print("\n\n====== COMPARISON TEST " .. test_name .. " ======\n")
        for file, paths in pairs(files_to_compare) do
            for idx, path in pairs(paths) do
                local result = ""
                if file_exists(path, file, actual_folder_name) then
                    local mismatch, line, actual_line, expected_line = compare_files_content(path, file, expected_folder_name, actual_folder_name)
                    if not mismatch then
                        result = "OK!"
                    else
                        result = "Mismatch on line " .. line .. "\nACTUAL:   " .. actual_line .. "\nEXPECTED: " .. expected_line
                        passed = false
                    end
                else
                    result = "FILE DOES NOT EXIST!"
                    passed = false
                end
                print(path .. file .. ": " .. result .. "\n")
            end
        end
        return passed
    end
end

local M = {}
M.create_file_comparison_test = create_file_comparison_test
return M


