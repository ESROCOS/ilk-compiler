--- Shared utilities related to the evaluation of the textual templates
--
-- The default options when loading a template is to use Xtend style templates.
-- No other option is set. When an options table is passed, no default values
-- for missing fields are applied.

local engine = require('template-text')


local function load(body, env, opts, included)
    local options = opts or {}
    options.xtendStyle = true -- we force it, the backend's templates use xtend style
    local ok, loaded = engine.tload(body, options, env, included)
    return ok, loaded
end

local function load_fail(body, env, opts, included)
    local ok,loaded = load(body, env, opts, included)
    if not ok then
        error("Failed to load the template:\n"..table.concat(loaded, "\n"), 2)
    end
    return loaded
end


local function eval(loaded, opts)
    local opts = opts or {}
    local ok, evaluated = loaded.evaluate(opts)
    return ok, evaluated
end

local function eval_fail(loaded, opts)
    local ok, evaluated = eval(loaded, opts)
    if not ok then
        error("Failed to evaluate the template:\n"..table.concat(evaluated, "\n"), 2)
    end
    return evaluated
end


local function load_eval(body, env, opts, included)
    local ok, ret = load(body, env, opts, included)
    if ok then
        ok, ret = eval(ret, opts)
    end
    return ok, ret
end

local function load_eval_fail(body, env, opts, included)
    local loaded = load_fail(body, env, opts, included)
    local ret    = eval_fail(loaded, opts)
    return ret
end



return {
    load = load,
    eval = eval,
    load_eval = load_eval,
    fail_on_error = {
        load = load_fail,
        eval = eval_fail,
        load_eval = load_eval_fail,
    },
}
