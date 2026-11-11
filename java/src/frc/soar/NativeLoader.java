package frc.soar;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

/** Loads libsoarbridge_jni exactly once, from java.library.path or from the jar. */
final class NativeLoader {
  private static final String LIBRARY = "soarbridge_jni";
  private static boolean loaded;

  private NativeLoader() {}

  static synchronized void load() {
    if (loaded) {
      return;
    }
    try {
      // On the robot the library is deployed next to the other native libraries.
      System.loadLibrary(LIBRARY);
    } catch (UnsatisfiedLinkError notOnPath) {
      loadFromJar(notOnPath);
    }
    loaded = true;
  }

  private static void loadFromJar(UnsatisfiedLinkError notOnPath) {
    String file = System.mapLibraryName(LIBRARY);
    String resource = "/native/" + platform() + "/" + file;
    try (InputStream in = NativeLoader.class.getResourceAsStream(resource)) {
      if (in == null) {
        UnsatisfiedLinkError error = new UnsatisfiedLinkError(
            file + " is not on java.library.path and " + resource + " is not in the jar");
        error.addSuppressed(notOnPath);
        throw error;
      }
      Path dir = Files.createTempDirectory("soarbridge");
      Path lib = dir.resolve(file);
      Files.copy(in, lib, StandardCopyOption.REPLACE_EXISTING);
      lib.toFile().deleteOnExit();
      dir.toFile().deleteOnExit();
      System.load(lib.toAbsolutePath().toString());
    } catch (IOException e) {
      UnsatisfiedLinkError error = new UnsatisfiedLinkError("Could not extract " + resource + ": " + e);
      error.addSuppressed(notOnPath);
      throw error;
    }
  }

  /** e.g. linux-amd64, linux-arm (roboRIO), linux-aarch64. */
  private static String platform() {
    String os = System.getProperty("os.name").toLowerCase();
    String name = os.contains("linux") ? "linux" : os.contains("mac") ? "osx" : "windows";
    return name + "-" + System.getProperty("os.arch");
  }
}
