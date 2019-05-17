local tests = {}
local creator = require('tests.file-comparison-test')
local creator2 = require('tests.error-expectation-test')
local psep = package.config:sub(1, 1) -- the path separator for the current OS

local basePath = "tests"..psep.."test_datasets"..psep
local function testDirs(testNum)
  local testDir  = "test"..testNum..psep
  return {
    input = basePath .. testDir .. "input",
    output= basePath .. testDir .. "output"
  }
end

local dirs = nil
local code = -1
local count = 0
local totNumberOfTests = 0

--tests[#tests + 1] = creator2.create_error_expectation_test('Should fail if an identifier is used which is neither a model constant nor is generated during computations', 'tests/test_datasets/test3/input')
--tests[#tests + 1] = creator2.create_error_expectation_test('Should fail if model constant is requested as computation output', 'tests/test_datasets/test2/input')

local cmpTests = {
   [1] = "Should not define a local variable for a transform requested as output (Eigen specific)"
  ,[2] = "Velocity solver code generation"
  ,[4] = "Should generate appropriate Inverse Kinematics computations"
  ,[5] = "Given an internally consistent ILK file, should generate compiling C++ code"
  ,[6] = "Should generate appropriate computations for a set of multiple solvers"
  ,[7] = "Should generate Python correctly"
}

for code, description in pairs(cmpTests) do
  dirs = testDirs(code)
  tests[code] = {}
  tests[code].description = description
  tests[code].cases, count = creator.fileComparisonTests(code, dirs.input, dirs.output)
  totNumberOfTests = totNumberOfTests + count
end


local passed_counter = 0

for code, testGroup in pairs(tests) do
  print("\n\n====== COMPARISON TEST " .. code .. " ======\n")
  print(testGroup.description)
  for backend, test in pairs(testGroup.cases) do
    local passed = test()
    if passed then
      passed_counter = passed_counter + 1
    end
  end
end

if passed_counter == totNumberOfTests then
    print("\n\n****** ALL TEST PASSED ******\n\n")
else
    print("\n\n------ " .. passed_counter .. "/" .. totNumberOfTests .. " tests passed. ------\n")
end
