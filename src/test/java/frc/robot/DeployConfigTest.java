package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.AutoConstants;
import swervelib.parser.SwerveParser;
import swervelib.parser.json.ModuleJson;

/**
 * Catches deploy-file mistakes (typos, mismatched geometry, missing paths or
 * named commands) at build time instead of on the robot.
 */
class DeployConfigTest {
  private static final File kDeployDir = new File("src/main/deploy");
  private static final File kPathPlannerDir = new File(kDeployDir, "pathplanner");
  private static final ObjectMapper kMapper = new ObjectMapper();

  private static JsonNode ppSettings;

  @BeforeAll
  static void load() throws IOException {
    SwerveParser.parseDirectory(new File(kDeployDir, "swerve"));
    ppSettings = kMapper.readTree(new File(kPathPlannerDir, "settings.json"));
  }

  @Test
  void swerveModulesAreValid() {
    assertEquals(4, SwerveParser.moduleJsons.length, "expected 4 swerve modules");
    for (ModuleJson module : SwerveParser.moduleJsons) {
      assertTrue(module.absoluteEncoderOffset >= 0 && module.absoluteEncoderOffset < 360,
          "absoluteEncoderOffset must be in [0, 360) degrees, got " + module.absoluteEncoderOffset);
    }
    assertTrue(SwerveParser.physicalPropertiesJson.gearing.drive.gearRatio > 1, "drive gearRatio");
    assertTrue(SwerveParser.physicalPropertiesJson.gearing.angle.gearRatio > 1, "angle gearRatio");
  }

  @Test
  void pathPlannerMatchesSwerveConfig() {
    var drive = SwerveParser.physicalPropertiesJson.gearing.drive;
    assertEquals(drive.gearRatio, ppSettings.get("driveGearing").asDouble(), 0.01,
        "PathPlanner driveGearing vs YAGSL drive gearRatio");
    assertEquals(Units.inchesToMeters(drive.diameter / 2), ppSettings.get("driveWheelRadius").asDouble(), 0.001,
        "PathPlanner driveWheelRadius vs YAGSL wheel diameter");

    // swervedrive.json module order is FL, FR, BL, BR
    String[] prefixes = {"fl", "fr", "bl", "br"};
    for (int i = 0; i < prefixes.length; i++) {
      ModuleJson module = SwerveParser.moduleJsons[i];
      assertEquals(Units.inchesToMeters(module.location.front),
          ppSettings.get(prefixes[i] + "ModuleX").asDouble(), 0.01, prefixes[i] + "ModuleX");
      assertEquals(Units.inchesToMeters(module.location.left),
          ppSettings.get(prefixes[i] + "ModuleY").asDouble(), 0.01, prefixes[i] + "ModuleY");
    }
  }

  @Test
  void pathPlannerRobotConfigLoads() throws Exception {
    RobotConfig.fromGUISettings();
  }

  @Test
  void autosReferenceExistingPathsAndNamedCommands() throws IOException {
    File[] files = concat(
        new File(kPathPlannerDir, "autos").listFiles((dir, name) -> name.endsWith(".auto")),
        new File(kPathPlannerDir, "paths").listFiles((dir, name) -> name.endsWith(".path")));
    for (File file : files) {
      List<JsonNode> commands = new ArrayList<>();
      collectCommands(kMapper.readTree(file), commands);
      for (JsonNode command : commands) {
        String type = command.get("type").asText();
        JsonNode data = command.path("data");
        if (type.equals("path")) {
          String pathName = data.get("pathName").asText();
          assertTrue(new File(kPathPlannerDir, "paths/" + pathName + ".path").exists(),
              file.getName() + " uses missing path '" + pathName + "'");
        } else if (type.equals("named")) {
          String name = data.get("name").asText();
          assertTrue(AutoConstants.kNamedCommands.contains(name),
              file.getName() + " uses unregistered named command '" + name + "'");
        }
      }
    }
  }

  // Every JSON object with a "type" and "data" field is a PathPlanner command
  private static void collectCommands(JsonNode node, List<JsonNode> out) {
    if (node.isObject() && node.has("type") && node.has("data")) {
      out.add(node);
    }
    node.forEach(child -> collectCommands(child, out));
  }

  private static File[] concat(File[] a, File[] b) {
    List<File> all = new ArrayList<>(List.of(a));
    all.addAll(List.of(b));
    return all.toArray(new File[0]);
  }
}
