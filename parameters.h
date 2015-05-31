// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <AP_Common.h>
#include "config.h"


class Parameters {
public:
    // Version info
    static const uint16_t k_format_version = EEPROM_FORMAT_VS;

    enum {
        k_param_format_version = 0,

        // Device objects
        k_param__INERT,
        k_param__INERT_NAV,
        k_param__AHRS,
        k_param__BARO,
        k_param__BARO_GLITCH,
        k_param__GPS,
        k_param__GPS_GLITCH,
        k_param__BAT,
        k_param__COMP,
        k_param__SON_RF,

        // PIDs
        k_param_PIT_RATE,
        k_param_ROL_RATE,
        k_param_PIT_STAB,
        k_param_ROL_STAB,
        k_param_YAW_RATE,
        k_param_YAW_STAB,
        k_param_THR_RATE,
        k_param_THR_STAB,
        k_param_ACC_RATE,
        k_param_ACC_STAB
    };

    // Version info
    AP_Int16 format_version;
};

extern const AP_Param::Info var_info[];
static Parameters g;

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define ASCALAR(v, name, def) { aparm.v.vtype, name, Parameters::k_param_ ## v, &aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, { group_info: class::var_info } }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &v, {group_info : class::var_info} }

#endif // PARAMETERS_H

