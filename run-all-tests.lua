local tests = {}
local creator = require('file-comparison-test')
local creator2 = require('error-expectation-test')

tests[#tests + 1] = creator.create_file_comparison_test('Main test')
tests[#tests + 1] = creator2.create_error_expectation_test('Should fail if model constant is requested as computation output', 'test_datasets/test2/input')
tests[#tests + 1] = creator.create_file_comparison_test('Should not generate a local declaration for a join-transform requested as output', 'test_datasets/test1/input', 'test_datasets/test1/output')
tests[#tests + 1] = creator.create_file_comparison_test('Given an internally consistent ILK file, should generate compiling C++ code', 'test_datasets/test5/input', 'test_datasets/test5/output')

local f = {}
f["ur5.cpp"] = { "" }
f["ur5.h"] = { "" }
f["ur5_fk_for_ik1_test.cpp"] = { "" }
f["ur5_ik1_test.cpp"] = { "" }
f["ur5_fk_for_ik1_timing.cpp"] = { "" }
f["ur5_ik1_timing.cpp"] = { "" }
f["makefile"] = { "" }
f["robot-defs.h"] = { "" }
tests[#tests + 1] = creator.create_file_comparison_test('Should generate appropriate Inverse Kinematics computations', 'test_datasets/test4/input', 'test_datasets/test4/output', nil, f)

local f = {}
f["ur5.cpp"] = { "" }
f["ur5.h"] = { "" }
f["ur5_fk1_test.cpp"] = { "" }
f["ur5_fk2_test.cpp"] = { "" }
f["ur5_ik1_test.cpp"] = { "" }
f["ur5_ik2_test.cpp"] = { "" }
f["ur5_fk__ik1_test.cpp"] = { "" }
f["ur5_fk1_timing.cpp"] = { "" }
f["ur5_fk2_timing.cpp"] = { "" }
f["ur5_ik1_timing.cpp"] = { "" }
f["ur5_ik2_timing.cpp"] = { "" }
f["ur5_fk__ik1_timing.cpp"] = { "" }
f["makefile"] = { "" }
f["robot-defs.h"] = { "" }
tests[#tests + 1] = creator.create_file_comparison_test('Should generate appropriate computations for a set of multiple solvers', 'test_datasets/test6/input', 'test_datasets/test6/output', nil, f)

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
