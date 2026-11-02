#include "rh56/persistent_store.hpp"

#include <filesystem>
#include <iostream>
#include <string>
#include <unistd.h>

namespace {

bool Check(bool condition, const char* message)
{
    if (!condition)
        std::cerr << message << '\n';
    return condition;
}

}  // namespace

int main()
{
    const auto directory = std::filesystem::temp_directory_path() /
                           ("rh56-pose-store-" + std::to_string(getpid()));
    const auto path = directory / "poses.tsv";
    std::filesystem::create_directories(directory);
    bool passed = true;
    try {
        rh56::PoseStore store(path);
        rh56::PoseRequest save;
        save.name = "thumb-safe";
        save.right = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
        save.left = {0.6f, 0.5f, 0.4f, 0.3f, 0.2f, 0.1f};
        save.delays_ms = {0, 0, 0, 0, 300, 350};
        const auto saved = store.Save(save);
        passed &= Check(!saved.id.empty(), "pose id was not generated");

        const auto renamed = store.Rename(saved.id, "thumb-late");
        passed &= Check(renamed && renamed->name == "thumb-late",
                        "pose rename failed");
        const auto delayed = store.SetDelays(saved.id, {0, 50, 100, 150, 400, 450});
        passed &= Check(delayed && delayed->delays_ms[4] == 400,
                        "pose delay update failed");

        rh56::PoseStore reloaded(path);
        const auto found = reloaded.Find(saved.id);
        passed &= Check(found && found->name == "thumb-late" &&
                            found->delays_ms[5] == 450,
                        "stored pose did not survive reload");
        passed &= Check(reloaded.Delete(saved.id), "pose delete failed");
        passed &= Check(!reloaded.Find(saved.id), "deleted pose still exists");

        const auto settings_path = directory / "settings.json";
        rh56::SettingsStore settings(settings_path);
        auto value = settings.Get();
        value.contact_threshold = 75;
        value.right_grip_force_grams = 350;
        settings.Set(value);
        rh56::SettingsStore reloaded_settings(settings_path);
        passed &= Check(reloaded_settings.Get().contact_threshold == 75 &&
                            reloaded_settings.Get().right_grip_force_grams == 350,
                        "settings did not survive reload");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        passed = false;
    }

    std::error_code error;
    std::filesystem::remove_all(directory, error);
    if (!passed)
        return 1;
    std::cout << "persistent_store_test: PASS\n";
    return 0;
}
