package frc.lib.util;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringArrayPublisher;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StringSubscriber;

/**
 * A chooser published in the same NetworkTables layout as the old SendableChooser, so Elastic's
 * "ComboBox Chooser" widget can both display and change it.
 *
 * <p>WPILib 2027's {@link org.wpilib.tunable.Selectable} publishes its selection as a robust
 * tunable ({@code selected/value} + {@code selected/tune}), which Elastic can't write to, so the
 * selection never reaches the robot.
 *
 * <p>Layout under the given path: {@code .type}, {@code options}, {@code default}, {@code active}
 * (published by the robot) and {@code selected} (written by the dashboard).
 *
 * <p>Call {@link #update()} periodically so the selection and {@code active} stay in sync.
 *
 * @param <V> The type of the values to be stored
 */
public final class DashboardChooser<V> {
  private final Map<String, V> m_map = new LinkedHashMap<>();
  private String m_defaultChoice = "";
  private String m_active = "";
  private Consumer<V> m_listener;

  private final StringPublisher m_typePub;
  private final StringArrayPublisher m_optionsPub;
  private final StringPublisher m_defaultPub;
  private final StringPublisher m_activePub;
  private final StringSubscriber m_selectedSub;

  /**
   * Creates a chooser and publishes it to NetworkTables.
   *
   * @param path NetworkTables path, e.g. "/Dashboard/Auto Chooser"
   */
  public DashboardChooser(String path) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(path);
    m_typePub = table.getStringTopic(".type").publish();
    m_typePub.set("String Chooser");
    m_optionsPub = table.getStringArrayTopic("options").publish();
    m_optionsPub.set(new String[0]);
    m_defaultPub = table.getStringTopic("default").publish();
    m_defaultPub.set("");
    m_activePub = table.getStringTopic("active").publish();
    m_activePub.set("");
    m_selectedSub = table.getStringTopic("selected").subscribe("");
  }

  /**
   * Adds the given object to the list of options.
   *
   * @param name the name of the option
   * @param object the option
   */
  public void add(String name, V object) {
    m_map.put(name, object);
    m_optionsPub.set(m_map.keySet().toArray(new String[0]));
  }

  /**
   * Adds the given object to the list of options and marks it as the default.
   *
   * @param name the name of the option
   * @param object the option
   */
  public void addDefault(String name, V object) {
    add(name, object);
    m_defaultChoice = name;
    m_defaultPub.set(name);
    update();
  }

  /**
   * Binds a listener that's called when the selected value changes.
   *
   * @param listener The function to call that accepts the new value
   */
  public void onChange(Consumer<V> listener) {
    m_listener = listener;
  }

  /** Reads the dashboard selection and republishes the active option. Call every loop. */
  public void update() {
    String selected = m_selectedSub.get();
    String active = m_map.containsKey(selected) ? selected : m_defaultChoice;
    if (!active.equals(m_active)) {
      m_active = active;
      m_activePub.set(active);
      V value = m_map.get(active);
      if (m_listener != null && value != null) {
        m_listener.accept(value);
      }
    }
  }

  /**
   * Returns the selected option, or the default if nothing valid is selected.
   *
   * @return the option selected, or null if there is no selection and no default
   */
  public V getSelected() {
    update();
    return m_map.get(m_active);
  }

  /**
   * Returns the name of the selected option.
   *
   * @return the active option name
   */
  public String getSelectedName() {
    update();
    return m_active;
  }
}
