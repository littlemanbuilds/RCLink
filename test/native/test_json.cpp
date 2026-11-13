#include <cstdlib>
#include <iostream>
#include <string>
#include "RcMacros.hpp"
#define ROLES(X) X(Throttle) X(Mode)
RC_DECLARE_ROLES(JsonRole,ROLES)
#include "Json.hpp"

static unsigned assertions=0;
#define CHECK(x) do { ++assertions; if (!(x)) { std::cerr << "FAIL " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (0)
int main() {
    for(const char *bad : {"null","true","false","0.5","\"0\"","\"oops\"","-1","32","2147483648"}) {
        rc::RcConfig<JsonRole> c; c.map(JsonRole::Mode,2);
        std::string json=std::string("{\"map\":{\"Mode\":1,\"Throttle\":")+bad+"}}";
        CHECK(!rc::load_json(JsonRole{},c,json.c_str()));
        CHECK(c.role_to_channel[1]==2); CHECK(!c.role_mapped[0]);
    }
    for(const char *field : {"\"raw\":[1000,2000,\"oops\"]", "\"raw\":[1000]", "\"raw\":null", "\"out\":[false,100]", "\"expo\":\"0.2\"", "\"invert\":1", "\"deadband_us\":0.5"}) {
        rc::RcConfig<JsonRole> c;
        std::string json=std::string("{\"axes\":{\"Throttle\":{")+field+"}}}";
        CHECK(!rc::load_json(JsonRole{},c,json.c_str()));
    }
    for(const char *field : {"\"values\":[0,\"oops\"]", "\"values\":[0,null]", "\"values\":[0,true]", "\"raw_levels\":[1000,2000.5]", "\"raw_levels\":[1000,\"2000\"]", "\"raw_levels\":[1000,32768]", "\"hyst_us\":true", "\"auto_levels\":0", "\"min_sep_us\":-1", "\"learn_alpha\":null"}) {
        rc::RcConfig<JsonRole> c;
        std::string json=std::string("{\"switches\":{\"Mode\":{")+field+"}}}";
        CHECK(!rc::load_json(JsonRole{},c,json.c_str()));
    }
    for(const char *json : {"null", "[]", "{\"map\":null}", "{\"axes\":[]}", "{\"switches\":{\"Mode\":null}}"}) {
        rc::RcConfig<JsonRole> c; CHECK(!rc::load_json(JsonRole{},c,json));
    }
    rc::RcConfig<JsonRole> c;
    CHECK(rc::load_json(JsonRole{},c,"{}"));
    CHECK(rc::load_json(JsonRole{},c,"{\"map\":{\"Unknown\":999,\"Throttle\":0,\"Mode\":31},\"axes\":{\"Throttle\":{\"raw\":[1000,2000,1500],\"out\":[-100,100],\"invert\":false}},\"switches\":{\"Mode\":{\"values\":[0,1.5],\"raw_levels\":[800,2200]}}}"));
    CHECK(c.role_to_channel[0]==0); CHECK(c.role_to_channel[1]==31); CHECK(rc::validate_config(c));
    CHECK(!rc::load_json(JsonRole{},c,nullptr));
    std::cout << "PASS JSON: " << assertions << " assertions\n";
}
