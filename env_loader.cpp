#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

void load_env_once()
{
    static bool loaded = false;
    if (loaded) return;
    loaded = true;

    std::filesystem::path dir = std::filesystem::current_path();

    while (true)
    {
        std::filesystem::path env = dir / ".env";

        if (std::filesystem::exists(env))
        {
            std::ifstream file(env);
            std::string line;

            while (std::getline(file, line))
            {
                if (line.empty() || line[0] == '#')
                    continue;

                auto pos = line.find('=');
                if (pos == std::string::npos)
                    continue;

                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 1);

                if (std::getenv(key.c_str()))
                    continue;

#ifdef _WIN32
                _putenv_s(key.c_str(), value.c_str());
#else
                setenv(key.c_str(), value.c_str(), 0);
#endif
            }

            return;
        }

        if (dir == dir.root_path())
            return;

        dir = dir.parent_path();
    }
}