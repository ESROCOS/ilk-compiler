local op_eigen = require('ilk.op-eigen')
local op_gsl = require('ilk.op-gsl')


local op_backend = {
    gsl = {
        var = op_gsl.var_op_gsl,
        assign = op_gsl.set_op_gsl,
        compose = op_gsl.compose_op_gsl,
        print = op_gsl.print_op_gsl,
        cleanup = op_gsl.cleanup_gsl
    },
    eigen = {
        var = op_eigen.var_op_eigen,
        assign = op_eigen.set_op_eigen,
        compose = op_eigen.compose_op_eigen,
        print = op_eigen.print_op_eigen,
        cleanup = function() end
    }
}

return op_backend