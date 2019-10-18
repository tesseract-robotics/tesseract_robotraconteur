#include <iostream>
#include <signal.h>
#include <RobotRaconteur.h>
#include <boost/program_options.hpp>

#include "robotraconteur_generated.h"
#include "tesseract_planner_impl.h"

#include "yaml-cpp/yaml.h"
#include <boost/locale.hpp>

using namespace RobotRaconteur;
using namespace tesseract_robotraconteur;

namespace po = boost::program_options;

static std::map<std::string,std::string> resource_buckets;

void load_resource_buckets()
{
    char* bucket_env_c = getenv("ROBOTRACONTEUR_BUCKET_FILES");
    if (!bucket_env_c) return;
    std::string bucket_env(bucket_env_c);

#ifdef _WIN32
    std::string pathsep = ";";
#else
    std::string pathsep = ":";
#endif

    std::vector<std::string> bucket_fnames;
    boost::split(bucket_fnames, bucket_env, boost::is_any_of(pathsep));
    for (auto bucket_fname : bucket_fnames)
    {
        YAML::Node bucket_file = YAML::LoadFile(bucket_fname);
        auto bucket_list = bucket_file["buckets"];
        for (YAML::const_iterator e=bucket_list.begin(); e!=bucket_list.end(); ++e)
        {
            auto bucket = *e;
            std::string bucket_name = bucket["name"].as<std::string>();
            std::string bucket_directory = bucket["directory"].as<std::string>();
            std::cout << "found bucket " << bucket["name"].as<std::string>() << " directory " << bucket_directory <<  std::endl;
            resource_buckets.insert(std::make_pair(bucket_name,bucket_directory));
        }
    }
}

std::string locator(const std::string& url)
{
    //Based on tesseract_rosutils/utils.h locateResource

     std::string mod_url = url;
    if (url.find("package://") == 0)
    {
        mod_url.erase(0, strlen("package://"));
        size_t pos = mod_url.find("/");
        if (pos == std::string::npos)
        {
            return std::string();
        }

        std::string package = mod_url.substr(0, pos);
        mod_url.erase(0, pos);

        auto e = resource_buckets.find(package);
        if (e == resource_buckets.end()) 
        {
            return std::string();
        }

        std::string package_path = e->second;

        if (package_path.empty())
        {
            return std::string();
        }

        mod_url = package_path + mod_url;
    }
    else if (url.find("file://") == 0)
    {
#ifdef _WIN32

    // Windows ASCII functions have an overly short path length of 255 characters. This
    // is easily exceeded with modern filesystems. Instead convert to wchar_t types
    // to use unicode which have a 2^16-1 length for the path.

    std::wstring url_w = boost::locale::conv::to_utf<wchar_t>(url,"Latin1");
    DWORD res = ::GetFullPathNameW(url_w.c_str(), 0, NULL, NULL);
    if (!res)
    {
        return std::string();
    }

    std::wstring file_w;
    file_w.resize((size_t)res);
    DWORD res2 = ::GetFullPathNameW(url_w.c_str(), (DWORD)file_w.size(), &file_w[0], NULL);
    if (res != res2)
    {
        return std::string();
    }

    mod_url = boost::locale::conv::from_utf(file_w,"Latin1");

#else
    mod_url.erase(0, strlen("file://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
    {
        return std::string();
    }
#endif
    }

    return mod_url;    
}

int main(int argc, char* argv[])
{

    load_resource_buckets();
    
    uint16_t rr_port;
    std::string rr_nodename;

    std::string urdf_fname;
    std::string srdf_fname;

    po::options_description generic_("Allowed options");
	generic_.add_options()
		("help,h", "print usage message")
        ("robotraconteur-nodename", po::value(&rr_nodename)->default_value("org.swri.tesseract"), "set nodename of tesseract service node")
		("robotraconteur-tcp-port", po::value(&rr_port)->default_value(63158), "set port for TcpTransport")
		("urdf-file", po::value(&urdf_fname), "URDF filename")
        ("srdf-file", po::value(&srdf_fname), "SRDF filename");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(generic_).run(), vm);
    po::notify(vm);

    std::ifstream urdf_t(urdf_fname);
    if (!urdf_t) throw RR::IOException("Could not open URDF file");
    std::stringstream urdf_buffer;
    urdf_buffer << urdf_t.rdbuf();
    
    std::ifstream srdf_t(srdf_fname);
    if (!srdf_t) throw RR::IOException("Could not open SRDF file");
    std::stringstream srdf_buffer;
    srdf_buffer << srdf_t.rdbuf();

    ServerNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES, rr_nodename, rr_port);

    auto obj = RR_MAKE_SHARED<TesseractPlannerImpl>();
    obj->Init(urdf_buffer.str(), srdf_buffer.str(), locator);

    RR::RobotRaconteurNode::s()->RegisterService("tesseract","com.robotraconteur.robotics.planning", obj);

    std::cerr << "Press enter quit" << std::endl;

    /*sigset_t   set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigprocmask( SIG_BLOCK, &set, NULL );
    int sig;
    sigwait(&set, &sig);*/

    getchar();

    std::cerr << "Goodbye!" << std::endl;

    return 0;
}
