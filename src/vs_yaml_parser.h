#ifndef __VS_YAML_PARSER__
#define __VS_YAML_PARSER__
#include <opencv2/opencv.hpp>
#include <string>

class YamlParser
{
public:
    YamlParser(const char* yaml_file = nullptr, const std::string& mode = "r")
    {
        if(yaml_file!=nullptr)
            open(yaml_file, mode);
    }

    bool open(const char* yaml_file, const std::string& mode)
    {
        if(mode == "r")
            m_fs.open(yaml_file, cv::FileStorage::READ);
        else if(mode == "w")
            m_fs.open(yaml_file, cv::FileStorage::WRITE);
        else
        {
            printf("[WARN]YamlParser: unknow mode '%s',"
                    "use 'r' or 'w' for read and write\n", mode.c_str());
            return false;
        }
        m_param_file = std::string(yaml_file);
        if(!m_fs.isOpened())
            printf("[WARN]YamlParser: cannot open param file '%s'\n", yaml_file);
        return m_fs.isOpened();
    }

    bool isOpened()
    {
        return m_fs.isOpened();
    }

    template<typename T>
    T read(const char* param_name, T default_val=T())
    {
        if(!m_fs.isOpened())
        {
            printf("[WARN]YamlParser: read faild, no yaml file open.\n");
            return default_val;
        }
        std::stringstream ss(param_name);
        cv::FileNode node;
        std::string t;
        bool first_node = true;
        while(getline(ss, t, '/'))
        {
            if(t.length() == 0) continue;
            if(first_node)
            {
                node = m_fs[t];
                first_node = false;
            }
            else
                node = node[t];
            if(node.isNone()) break;
        }
        if(node.isNone())
        {
            printf("[WARN]YamlParser: param '%s' not found, use default val.\n", param_name);
            return default_val;
        }
        else
        {
            T a;
            node >> a;
            // printf("read %s: ", param_name);
            // std::cout<<a<<std::endl;
            return a;
        }
    }

    template<typename T>
    void write(const char* param_name, T val)
    {
        if(!m_fs.isOpened())
        {
            printf("[WARN]YamlParser: write faild, no yaml file open.\n");
            return;
        }
        m_fs<<param_name<<val;
    }

    cv::FileStorage& fs() {return m_fs;}

private:
    cv::FileStorage     m_fs;
    std::string         m_param_file;
};

#endif//__VS_YAML_PARSER__