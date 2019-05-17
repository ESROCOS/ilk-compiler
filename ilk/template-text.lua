
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

--- Copy every string in the second argument in the first, prepending indentation.
local insertTableContents = function(text, lines, totIndent)
  for i,line in ipairs(lines) do
    if line ~= "" then
      table.insert(text, totIndent .. line)
    else
      table.insert(text, "")
    end
  end
end

--- Evaluate the given text-template into a string.
-- Regular text in the template is copied verbatim, while expressions in the
-- form $(<var>) are replaced with the textual representation of <var>, which
-- must be defined in the given environment.
-- Finally, lines starting with @ are interpreted entirely as Lua code.
--
-- @param template the text-template, as a string
-- @param env the environment for the evaluation of the expressions in the
--        templates (if not given, 'table', 'pairs', 'ipairs' are added
--        automatically to this enviroment)
-- @param opts non-mandatory options
--        - verbose: true, for printing an error message in case of failure
--        - indent: number of blanks to be prepended before every output line;
--          this applies to the whole template, relative indentation between
--          different lines is preserved
--        - xtendStyle: if true, variables are matched with this pattern "«<var>»"
-- @return The text of the evaluated template; if the option 'returnTable' is
--         set to true, though, the table with the sequence of lines of text is
--         returned instead
local function template_eval(template, env, opts)

  local opts    = opts or {}
  local verbose = opts.verbose or false
  local indent  = string.format("%s", string.rep(' ', (opts.indent or 0) ) )

  -- Define the matching patter for the variables, depending on options.
  -- The matching pattern reads in general as: <text><var><string position>
  local varMatch = {
    pattern = "(.-)$(%b())()",
    extract = function(expr) return expr:sub(2,-2) end
  }
  if opts.xtendStyle then
    varMatch.pattern = "(.-)«(.-)»()"
    varMatch.extract = function(expr) return expr end
  end

  -- Generate a line of code for each line in the input template.
  -- The lines of code are also strings; we add them in the 'chunk' table.
  -- Every line is either the insertion in a table of a string, or a 1-to-1 copy
  --  of the code inserted in the template via the '@' character.
  local chunk = {"local text={}"}
  local lineOfCode = nil
  for line in lines(template) do
    -- Look for a '@' ignoring blanks (%s) at the beginning of the line
    -- If it's there, copy the string following the '@'
    local s,e = line:find("^%s*@")
    if s then
      lineOfCode = line:sub(e+1)
    else
      -- Look for the specials '${..}', which must be alone in the line
      local tableIndent, tableVarName = line:match("^([%s]*)${(.-)}[%s]*")
      if tableVarName ~= nil then
        -- Preserve the indentation used for the "${..}" in the original template.
        -- "Sum" it to the global indentation passed here as an option.
        local totIndent = string.format("%q", indent .. tableIndent)
        lineOfCode = "__insertTableContents(text, " .. tableVarName .. ", " .. totIndent .. ")"
      else
        -- Look for the template variables in the current line.
        -- All the matches are stored as strings '"<text>" .. <variable>'
        -- Note that <text> can be empty
        local subexpr = {}
        local lastindex = 1
        local c = 1
        for text, expr, index in line:gmatch(varMatch.pattern) do
          subexpr[c] = string.format("%q .. %s", text, varMatch.extract(expr))
          lastindex = index
          c = c + 1
        end
        -- Add the remaining part of the line (no further variable) - or the
        -- entire line if no $() was found
        subexpr[c] = string.format("%q", line:sub(lastindex))

        -- Concatenate the subexpressions into a single one, prepending the
        -- indentation if it is not empty
        local expression = table.concat(subexpr, ' .. ')
        if expression~="\"\"" and indent~="" then
          expression = string.format("%q", indent) .. ' .. ' .. expression
        end

        lineOfCode = "table.insert(text, " .. expression .. ")"
      end
    end
    table.insert(chunk, lineOfCode)
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
  env.__insertTableContents = insertTableContents
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





