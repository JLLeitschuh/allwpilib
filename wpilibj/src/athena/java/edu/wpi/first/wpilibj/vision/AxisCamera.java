/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2014-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.vision;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.HSLImage;
import edu.wpi.first.wpilibj.image.NIVisionException;

import static com.ni.vision.NIVision.Image;
import static com.ni.vision.NIVision.Priv_ReadJPEGString_C;

/**
 * Axis M1011 network camera.
 */
public class AxisCamera {
  public enum WhiteBalance {
    kAutomatic, kHold, kFixedOutdoor1, kFixedOutdoor2, kFixedIndoor, kFixedFluorescent1,
    kFixedFluorescent2,
  }

  public enum ExposureControl {
    kAutomatic, kHold, kFlickerFree50Hz, kFlickerFree60Hz,
  }

  public enum Resolution {
    k640x480, k480x360, k320x240, k240x180, k176x144, k160x120,
  }

  public enum Rotation {
    k0, k180
  }

  private static final String[] kWhiteBalanceStrings = {"auto", "hold", "fixed_outdoor1",
      "fixed_outdoor2", "fixed_indoor", "fixed_fluor1", "fixed_fluor2",};

  private static final String[] kExposureControlStrings = {"auto", "hold", "flickerfree50",
      "flickerfree60",};

  private static final String[] kResolutionStrings = {"640x480", "480x360", "320x240", "240x180",
      "176x144", "160x120",};

  private static final String[] kRotationStrings = {"0", "180",};

  private static final int kImageBufferAllocationIncrement = 1000;

  private String cameraHost;
  private Socket cameraSocket;

  private ByteBuffer imageData = ByteBuffer.allocate(5000);
  private final Object imageDataLock = new Object();
  private boolean freshImage = false;

  private int brightness = 50;
  private WhiteBalance whiteBalance = WhiteBalance.kAutomatic;
  private int colorLevel = 50;
  private ExposureControl exposureControl = ExposureControl.kAutomatic;
  private int exposurePriority = 50;
  private int maxFPS = 0;
  private Resolution resolution = Resolution.k640x480;
  private int compression = 50;
  private Rotation rotation = Rotation.k0;
  private final Object parametersLock = new Object();
  private boolean parametersDirty = true;
  private boolean streamDirty = true;

  private boolean done = false;

  /**
   * AxisCamera constructor.
   *
   * @param cameraHost The host to find the camera at, typically an IP address
   */
  public AxisCamera(String cameraHost) {
    cameraHost = cameraHost;
    captureThread.start();
  }

  /**
   * Return true if the latest image from the camera has not been retrieved by calling GetImage()
   * yet.
   *
   * @return true if the image has not been retrieved yet.
   */
  public boolean isFreshImage() {
    return freshImage;
  }

  /**
   * Get an image from the camera and store it in the provided image.
   *
   * @param image The imaq image to store the result in. This must be an HSL or RGB image.
   * @return <code>true</code> upon success, <code>false</code> on a failure
   */
  public boolean getImage(Image image) {
    if (imageData.limit() == 0) {
      return false;
    }

    synchronized (imageDataLock) {
      Priv_ReadJPEGString_C(image, imageData.array());
    }

    freshImage = false;

    return true;
  }

  /**
   * Get an image from the camera and store it in the provided image.
   *
   * @param image The image to store the result in. This must be an HSL or RGB image
   * @return true upon success, false on a failure
   */
  public boolean getImage(ColorImage image) {
    return this.getImage(image.image);
  }

  /**
   * Instantiate a new image object and fill it with the latest image from the camera.
   *
   * @return a pointer to an HSLImage object
   */
  public HSLImage getImage() throws NIVisionException {
    HSLImage image = new HSLImage();
    this.getImage(image);
    return image;
  }

  /**
   * Request a change in the brightness of the camera images.
   *
   * @param brightness valid values 0 .. 100
   */
  public void writeBrightness(int brightness) {
    if (brightness < 0 || brightness > 100) {
      throw new IllegalArgumentException("Brightness must be from 0 to 100");
    }

    synchronized (parametersLock) {
      if (brightness != brightness) {
        brightness = brightness;
        parametersDirty = true;
      }
    }
  }

  /**
   * The brightness.
   *
   * @return The configured brightness of the camera images
   */
  public int getBrightness() {
    synchronized (parametersLock) {
      return brightness;
    }
  }

  /**
   * Request a change in the white balance on the camera.
   *
   * @param whiteBalance Valid values from the <code>WhiteBalance</code> enum.
   */
  public void writeWhiteBalance(WhiteBalance whiteBalance) {
    synchronized (parametersLock) {
      if (whiteBalance != whiteBalance) {
        whiteBalance = whiteBalance;
        parametersDirty = true;
      }
    }
  }

  /**
   * The white balance.
   *
   * @return The configured white balances of the camera images
   */
  public WhiteBalance getWhiteBalance() {
    synchronized (parametersLock) {
      return whiteBalance;
    }
  }

  /**
   * Request a change in the color level of the camera images.
   *
   * @param colorLevel valid values are 0 .. 100
   */
  public void writeColorLevel(int colorLevel) {
    if (colorLevel < 0 || colorLevel > 100) {
      throw new IllegalArgumentException("Color level must be from 0 to 100");
    }

    synchronized (parametersLock) {
      if (colorLevel != colorLevel) {
        colorLevel = colorLevel;
        parametersDirty = true;
      }
    }
  }

  /**
   * The current color level.
   *
   * @return The configured color level of the camera images
   */
  public int getColorLevel() {
    synchronized (parametersLock) {
      return colorLevel;
    }
  }

  /**
   * Request a change in the camera's exposure mode.
   *
   * @param exposureControl A mode to write in the <code>Exposure</code> enum.
   */
  public void writeExposureControl(ExposureControl exposureControl) {
    synchronized (parametersLock) {
      if (exposureControl != exposureControl) {
        exposureControl = exposureControl;
        parametersDirty = true;
      }
    }
  }

  /**
   * The current exposure control.
   *
   * @return The configured exposure control mode of the camera
   */
  public ExposureControl getExposureControl() {
    synchronized (parametersLock) {
      return exposureControl;
    }
  }

  /**
   * Request a change in the exposure priority of the camera.
   *
   * @param exposurePriority Valid values are 0, 50, 100. 0 = Prioritize image quality 50 = None 100
   *                         = Prioritize frame rate
   */
  public void writeExposurePriority(int exposurePriority) {
    if (exposurePriority != 0 && exposurePriority != 50 && exposurePriority != 100) {
      throw new IllegalArgumentException("Exposure priority must be 0, 50, or 100");
    }

    synchronized (parametersLock) {
      if (exposurePriority != exposurePriority) {
        exposurePriority = exposurePriority;
        parametersDirty = true;
      }
    }
  }

  /**
   * Gets the exposure priority.
   *
   * @return The configured exposure priority of the camera
   */
  public int getExposurePriority() {
    synchronized (parametersLock) {
      return exposurePriority;
    }
  }

  /**
   * Write the maximum frames per second that the camera should send Write 0 to send as many as
   * possible.
   *
   * @param maxFPS The number of frames the camera should send in a second, exposure permitting.
   */
  public void writeMaxFPS(int maxFPS) {
    synchronized (parametersLock) {
      if (maxFPS != maxFPS) {
        maxFPS = maxFPS;
        parametersDirty = true;
        streamDirty = true;
      }
    }
  }

  /**
   * The max frames per second of the camera.
   *
   * @return The configured maximum FPS of the camera
   */
  public int getMaxFPS() {
    synchronized (parametersLock) {
      return maxFPS;
    }
  }

  /**
   * Write resolution value to camera.
   *
   * @param resolution The camera resolution value to write to the camera.
   */
  public void writeResolution(Resolution resolution) {
    synchronized (parametersLock) {
      if (resolution != resolution) {
        resolution = resolution;
        parametersDirty = true;
        streamDirty = true;
      }
    }
  }

  /**
   * Gets the configured resolution (not necessarily the same resolution as the most recent image,
   * if it was changed recently).
   *
   * @return The configured resolution of the camera.
   */
  public Resolution getResolution() {
    synchronized (parametersLock) {
      return resolution;
    }
  }

  /**
   * Write the compression value to the camera.
   *
   * @param compression Values between 0 and 100.
   */
  public void writeCompression(int compression) {
    if (compression < 0 || compression > 100) {
      throw new IllegalArgumentException("Compression must be from 0 to 100");
    }

    synchronized (parametersLock) {
      if (compression != compression) {
        compression = compression;
        parametersDirty = true;
        streamDirty = true;
      }
    }
  }

  /**
   * Gets the configured compression level of the camera images.
   *
   * @return The configured compression level of the camera images.
   */
  public int getCompression() {
    synchronized (parametersLock) {
      return compression;
    }
  }

  /**
   * Write the rotation value to the camera. If you mount your camera upside down, use this to
   * adjust the image for you.
   *
   * @param rotation A value from the {@link Rotation} enum
   */
  public void writeRotation(Rotation rotation) {
    synchronized (parametersLock) {
      if (rotation != rotation) {
        this.rotation = rotation;
        parametersDirty = true;
        streamDirty = true;
      }
    }
  }

  /**
   * Gets the configured rotation mode of the camera.
   *
   * @return The configured rotation mode of the camera
   */
  public Rotation getRotation() {
    synchronized (parametersLock) {
      return rotation;
    }
  }

  /**
   * Thread spawned by AxisCamera constructor to receive images from cam.
   */
  private Thread captureThread = new Thread(new Runnable() {
    @Override
    public void run() {
      int consecutiveErrors = 0;

      // Loop on trying to setup the camera connection. This happens in a
      // background
      // thread so it shouldn't effect the operation of user programs.
      while (!done) {
        String requestString =
            "GET /mjpg/video.mjpg HTTP/1.1\n" + "User-Agent: HTTPStreamClient\n"
                + "Connection: Keep-Alive\n" + "Cache-Control: no-cache\n"
                + "Authorization: Basic RlJDOkZSQw==\n\n";

        try {
          cameraSocket = AxisCamera.this.createCameraSocket(requestString);
          AxisCamera.this.readImagesFromCamera();
          consecutiveErrors = 0;
        } catch (IOException ex) {
          consecutiveErrors++;

          if (consecutiveErrors > 5) {
            ex.printStackTrace();
          }
        }

        Timer.delay(0.5);
      }
    }
  });

  /**
   * This function actually reads the images from the camera.
   */
  private void readImagesFromCamera() throws IOException {
    DataInputStream cameraInputStream = new DataInputStream(cameraSocket.getInputStream());

    while (!done) {
      String line = cameraInputStream.readLine();

      if (line.startsWith("Content-Length: ")) {
        int contentLength = Integer.valueOf(line.substring(16));

        /* Skip the next blank line */
        cameraInputStream.readLine();
        contentLength -= 4;

        /* The next four bytes are the JPEG magic number */
        byte[] data = new byte[contentLength];
        cameraInputStream.readFully(data);

        synchronized (imageDataLock) {
          if (imageData.capacity() < data.length) {
            imageData = ByteBuffer.allocate(data.length + kImageBufferAllocationIncrement);
          }

          imageData.clear();
          imageData.limit(contentLength);
          imageData.put(data);

          freshImage = true;
        }

        if (this.writeParameters()) {
          break;
        }

        /* Skip the boundary and Content-Type header */
        cameraInputStream.readLine();
        cameraInputStream.readLine();
      }
    }

    cameraSocket.close();
  }

  /**
   * Send a request to the camera to set all of the parameters. This is called in the capture thread
   * between each frame. This strategy avoids making lots of redundant HTTP requests, accounts for
   * failed initial requests, and avoids blocking calls in the main thread unless necessary.
   *
   * <p>This method does nothing if no parameters have been modified since it last completely
   * successfully.
   *
   * @return <code>true</code> if the stream should be restarted due to a parameter changing.
   */
  private boolean writeParameters() {
    if (parametersDirty) {
      String request = "GET /axis-cgi/admin/param.cgi?action=update";

      synchronized (parametersLock) {
        request += "&ImageSource.I0.Sensor.Brightness=" + brightness;
        request +=
            "&ImageSource.I0.Sensor.WhiteBalance=" + kWhiteBalanceStrings[whiteBalance.ordinal()];
        request += "&ImageSource.I0.Sensor.ColorLevel=" + colorLevel;
        request +=
            "&ImageSource.I0.Sensor.Exposure="
                + kExposureControlStrings[exposureControl.ordinal()];
        request += "&ImageSource.I0.Sensor.ExposurePriority=" + exposurePriority;
        request += "&Image.I0.Stream.FPS=" + maxFPS;
        request += "&Image.I0.Appearance.Resolution=" + kResolutionStrings[resolution.ordinal()];
        request += "&Image.I0.Appearance.Compression=" + compression;
        request += "&Image.I0.Appearance.Rotation=" + kRotationStrings[rotation.ordinal()];
      }

      request += " HTTP/1.1\n";
      request += "User-Agent: HTTPStreamClient\n";
      request += "Connection: Keep-Alive\n";
      request += "Cache-Control: no-cache\n";
      request += "Authorization: Basic RlJDOkZSQw==\n\n";

      try {
        Socket socket = this.createCameraSocket(request);
        socket.close();

        parametersDirty = false;

        if (streamDirty) {
          streamDirty = false;
          return true;
        } else {
          return false;
        }
      } catch (IOException | NullPointerException ex) {
        return false;
      }

    }

    return false;
  }

  /**
   * Create a socket connected to camera Used to create a connection for reading images and setting
   * parameters
   *
   * @param requestString The initial request string to send upon successful connection.
   * @return The created socket
   */
  private Socket createCameraSocket(String requestString) throws IOException {
    /* Connect to the server */
    Socket socket = new Socket();
    socket.connect(new InetSocketAddress(cameraHost, 80), 5000);

    /* Send the HTTP headers */
    OutputStream socketOutputStream = socket.getOutputStream();
    socketOutputStream.write(requestString.getBytes());

    return socket;
  }

  @Override
  public String toString() {
    return "AxisCamera{" + "FreshImage=" + isFreshImage() + ", Brightness=" + getBrightness()
        + ", WhiteBalance=" + getWhiteBalance() + ", ColorLevel=" + getColorLevel()
        + ", ExposureControl=" + getExposureControl() + ", ExposurePriority="
        + getExposurePriority() + ", MaxFPS=" + getMaxFPS() + ", Resolution=" + getResolution()
        + ", Compression=" + getCompression() + ", Rotation=" + getRotation() + '}';
  }
}
