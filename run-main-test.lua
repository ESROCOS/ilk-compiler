local tests = {}
local creator = require('file-comparison-test')
test = creator.create_file_comparison_test()
tests[#tests + 1] = test

local passed_counter = 0
for i, test in pairs(tests) do
    local passed = test()
    if passed then
        passed_counter = passed_counter + 1
    end
end

if passed_counter == #tests then
    print("\n\n****** ALL TEST PASSED ******\n\n")
else
    print("\n\n------ " .. passed_counter .. "/" .. (#tests) .. " tests passed. ------\n")
end
