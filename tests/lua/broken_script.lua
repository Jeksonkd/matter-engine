-- Intentionally throws inside on_start to verify ScriptEngine reports the
-- error via onError instead of crashing the host process.
function on_start(body)
    error("intentional failure for script_test")
end
