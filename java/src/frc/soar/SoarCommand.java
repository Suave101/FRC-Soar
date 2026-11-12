package frc.soar;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * One command from the Soar agent's output-link, for any command name.
 *
 * <p>A command is an identifier {@code ^<name> <cmd>} on the output-link; its
 * children are the parameters. Nested identifiers are flattened into dotted
 * keys, so {@code (<cmd> ^target <t>) (<t> ^x 1.5)} is the key {@code target.x}.
 * Values are {@link Long} (Soar int), {@link Double} (Soar float) or
 * {@link String}. Soar has no boolean: booleans are the strings
 * {@code "true"} and {@code "false"}.
 */
public final class SoarCommand {
  private final String name;
  private final long id;
  private final Map<String, Object> params;

  SoarCommand(String name, long id, Map<String, Object> params) {
    this.name = name;
    this.id = id;
    this.params = Collections.unmodifiableMap(params);
  }

  /** Copies a native command; does not take ownership of it. */
  static SoarCommand from(Command command) {
    Map<String, Object> params = new LinkedHashMap<>();
    StringVector names = command.paramNames();
    try {
      for (int i = 0; i < names.size(); i++) {
        String key = names.get(i);
        Value value = command.getParam(key);
        try {
          switch (value.getType()) {
            case INT:
              params.put(key, value.getIntValue());
              break;
            case FLOAT:
              params.put(key, value.getFloatValue());
              break;
            default:
              params.put(key, value.getStringValue());
              break;
          }
        } finally {
          value.delete();
        }
      }
    } finally {
      names.delete();
    }
    return new SoarCommand(command.getName(), command.getId(), params);
  }

  /** Output-link attribute, e.g. {@code ground-intake}. */
  public String name() {
    return name;
  }

  /** Identifies the command on the output-link. */
  public long id() {
    return id;
  }

  /** Every parameter except {@code ^status}, by (dotted) key. */
  public Map<String, Object> params() {
    return params;
  }

  public boolean has(String key) {
    return params.containsKey(key);
  }

  /** An int or float parameter, or a string that is a number; else {@code fallback}. */
  public double getDouble(String key, double fallback) {
    Object v = params.get(key);
    if (v instanceof Number) {
      return ((Number) v).doubleValue();
    }
    if (v instanceof String) {
      try {
        return Double.parseDouble((String) v);
      } catch (NumberFormatException e) {
        return fallback;
      }
    }
    return fallback;
  }

  public double getDouble(String key) {
    return getDouble(key, 0.0);
  }

  /** The parameter as Soar prints it, or {@code fallback} if missing. */
  public String getString(String key, String fallback) {
    Object v = params.get(key);
    return v == null ? fallback : v.toString();
  }

  public String getString(String key) {
    return getString(key, "");
  }

  /** {@code true}/{@code false}, or 1/0; else {@code fallback}. */
  public boolean getBoolean(String key, boolean fallback) {
    Object v = params.get(key);
    if ("true".equals(v) || "1".equals(v) || Long.valueOf(1).equals(v)) {
      return true;
    }
    if ("false".equals(v) || "0".equals(v) || Long.valueOf(0).equals(v)) {
      return false;
    }
    return fallback;
  }

  @Override
  public String toString() {
    StringBuilder s = new StringBuilder("^").append(name);
    for (Map.Entry<String, Object> p : params.entrySet()) {
      s.append(" ^").append(p.getKey()).append(' ').append(p.getValue());
    }
    return s.toString();
  }
}
