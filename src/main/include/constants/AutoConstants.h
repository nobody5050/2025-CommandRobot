#pragma once

#include <subzero/frc/smartdashboard/TaggedChooser.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include <vector>
#include <string>
#include <map>
#include <frc/trajectory/TrapezoidProfile.h>

namespace AutoConstants
{
    // List all your autos here
    enum class AutoType
    {
        EmptyAuto = 0,
        DemoAuto,
        ComplexAuto
    };

    // Associate an auto with it's name in the sendable chooser, and with it's tags
    static const std::vector<subzero::TaggedChooser<AutoType>::TaggedChooserEntry> kChooserEntries{
        {{AutoType::EmptyAuto, "Empty Auto"}, {"0 coral", "0 algae", "minimal"}},
        {{AutoType::DemoAuto, "Demo Auto"}, {"0 coral", "0 algae", "minimal"}},
        {{AutoType::ComplexAuto, "Complex Auto"}, {"2 coral", "0 algae", "decent"}}};

    // List all tags here to filter by on the dashboard
    static const std::vector<subzero::TaggedChooser<AutoType>::TaggedChooserSelectorGroup>
        kChooserGroups{
            {"Coral Count", {"0 coral", "1 coral", "2 coral", "3 coral", "4 coral"}},
            {"Algae Count", {"0 algae", "1 algae", "2 algae"}},
            {"Impact", {"minimal", "decent", "high"}},
        };

    // Associate an auto with it's file name
    static const std::map<AutoType, std::string> kPpAutos = {
        {AutoType::DemoAuto, "Demo Auto"},
        {AutoType::ComplexAuto, "Complex Auto"}};

    extern const frc::TrapezoidProfile<units::meter>::Constraints
        kLinearAxisConstraints;
} // namespace AutoConstants