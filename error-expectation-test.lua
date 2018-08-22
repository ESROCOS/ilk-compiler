local function default_input_folder_name()
    return "etc/example"
end


-- selects the generator from the backend
local function create_error_expectation_test(name, input_folder)
    local test_name = name or default_test_name()
    local input_folder_name = input_folder or default_input_folder_name()

    return function()
        local passed = true
        os.execute('rm -rf out')
        print("\n\n====== ERROR EXPECTED TEST: " .. test_name .. " ======\n")
        local was_error = os.execute('./ilk-compiler.lua -b eigen --indir ' .. input_folder_name .. ' --outdir out --silent')

        print(was_error)
        if was_error == true then
            print("TEST FAILED! The script completed successfully despite being expected to fail.")
            passed = false
        else
            print("TEST OK! The script threw an error as expected.")
        end

        return passed
    end
end

local M = {}
M.create_error_expectation_test = create_error_expectation_test
return M


