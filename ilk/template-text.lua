
--- Evaluate a chunk of code in a constrained environment.
-- @param unsafe_code code string
-- @param optional environment table.
-- @return true or false depending on success
-- @return function or error message
local function eval_sandbox(unsafe_code, env)
  local env = env or {}
  --  print(unsafe_code)
  local unsafe_fun, msg = load(unsafe_code, nil, 't', env)
  if not unsafe_fun then return false, msg end
  return xpcall(unsafe_fun, debug.traceback)
end

local function lines(s)
        if s:sub(-1)~="\n" then s=s.."\n" end
        return s:gmatch("(.-)\n")
end

--- Evaluate the given text-template into a string.
-- Regular text in the template is copied verbatim, while expressions in the
-- form $(<var>) are replaced with the textual representation of <var>, which
-- must be defined in the given environment.
-- Finally, lines starting with @ are interpreted entirely as Lua code.
--
-- @param template the text-template, as a string
-- @param env the environment for the evaluation of the expressions in the
--        templates (if not given, 'table', 'pairs', 'ipairs' are added automatically
--        to this enviroment)
-- @param opts non-mandatory options
--        - verbose: true, for printing an error message in case of failure
--        - indent: number of blanks to be prepended before any output line
-- @return The text of the evaluated template
local function template_eval(template, env, opts)
  -- Generate a line of code for each line in the input template.
  -- The lines of code are also strings; we add them in the 'chunk' table.
  -- Every line is either the insertion in a table of a string, or a 1-to-1 copy
  --  of the code inserted in the template via the '@' character.
  local opts    = opts or {}
  local verbose = opts.verbose or false
  local indent  = string.format("%q", string.rep(' ', (opts.indent or 0) ) )

  local varMatch = {
    pattern = "(.-)$(%b())()",
    extract = function(expr) return expr:sub(2,-2) end
  }
  if opts.xtendStyle then
    varMatch.pattern = "(.-)«(.-)»()"
    varMatch.extract = function(expr) return expr end
  end

  local chunk    = {"local text={}"}
  for line in lines(template) do
    -- Look for a '@' ignoring blanks (%s) at the beginning of the line
    -- If it's there, copy the string following the '@'
    local s,e = line:find("^%s*@")
    if s then
      table.insert(chunk, line:sub(e+1))
    else
      -- Look for the specials '${..}', which must be alone in the line
      local c1,c2 = line:match("^([%s]*)${(.-)}[%s]*")
      if c2 ~= nil then
        table.insert(chunk, "for i,v in ipairs(" .. c2 .. ") do")
        table.insert(chunk, string.format("table.insert(text, %s..%q..v)", indent,c1) )
        table.insert(chunk, "end")
      else
        -- Look for the specials '$(..)'
        -- The matching pattern reads as: <text><dollar var><string position>
        -- Note that <text> can be empty
        -- All the matching pairs are stored as strings '"<text>" .. <dollar var>'
        local subexpr = {}
        local lastindex = 1
        local c = 1
        for text, expr, index in line:gmatch(varMatch.pattern) do
          subexpr[c] = string.format("%q .. %s", text, varMatch.extract(expr))
          lastindex = index
          c = c + 1
        end
        -- add the remaining part of the line (no further $() ) - or the entire line if no $() was found
        subexpr[c] = string.format("%q", line:sub(lastindex))

        -- Concatenate the subexpressions into a single one, prepending the
        -- indentation if it is not empty
        local expression = table.concat(subexpr, ' .. ')
        if expression~="\"\"" and indent~="\"\"" then
          expression = indent .. ' .. ' .. expression
        end
        --print(expression)
        table.insert(chunk, "table.insert(text, " .. expression .. ")")
      end
    end
  end

  local returnTable = opts.returnTable or false
  if returnTable then
    table.insert(chunk, "return text")
  else
    -- The last line of code performs string concatenation, so that the evaluation
    -- of the code eventually leads to a string
    table.insert(chunk, "return table.concat(text, '\\n')")
  end
    --print( table.concat(chunk, '\n') )


  env.table = (env.table or table)
  env.pairs = (env.pairs or pairs)
  env.ipairs = (env.ipairs or ipairs)
  local ret, str = eval_sandbox(table.concat(chunk, '\n'), env)
  if not ret and verbose then
    local linen = tonumber(str:match(".-\"local text={}...\"]:(%d+).*"))
    local line = "??"
    if linen ~= nil then line = chunk[linen] end
    local errmsg = "template_eval() failed around this line:\n\t>>> " .. line .. "\n\t" .. str
    return ret, errmsg
  end
  return ret, str
end


local test = function(env, opts)
  local tpl = [[
text text text only
text $(var1) text

$(var1) text text text $(var2)
$(var2) «var1» «var2»

-- table begin
${atable}
-- table end
text text text $(var2)
$(var2) text text text
$(var1)$(var1)
]]

env.var1 = env.var1 or "__DEF1"
env.var2 = env.var2 or "__DEF2"
env.atable = env.atable or {"line1", "line2", "", "line4"}

local ok,res = template_eval(tpl, env, opts)
return res

end

return {template_eval = template_eval, test=test}





