--[[ Backend-specific templates ]] --
local main_header_gsl = [[
/* This file is autogenerated by 'gen-offline'
 *   2017, Enea Scioni, KU Leuven, Belgium
 *
*/
#include <stdio.h>

#include "grc_gsl/grc_gsl_types.h"
#include "grc_gsl/position_vector.h"
#include "grc_gsl/rotation_matrix.h"
#include "grc_gsl/transformation_matrix.h"

int main(int argc, char** argv) {

]]

local main_footer_gsl = [[
      return 0;
}
]]



local M = {
    header = main_header_gsl,
    footer = main_footer_gsl
}

return M