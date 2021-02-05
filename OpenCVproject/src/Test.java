import java.awt.image.BufferedImage;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import javafx.application.Application;
import javafx.embed.swing.SwingFXUtils;
import javafx.scene.Scene;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class Test extends Application {
    @Override
    public void start(Stage stage) throws Exception {
        Pane pane = new Pane();
        Scene scene = new Scene(pane, 500,500);

        String path = "C:\\RoboticsCode\\UltimateGoal4042\\OpenCVproject\\img\\ceilingfan.png";
        WritableImage image = getProcessedImage(path);
        ImageView imageView = new ImageView(image);

        pane.getChildren().add(imageView);

        stage.setScene(scene);
        stage.setTitle("opencv");
        stage.show();
    }


    public static void main(String[] args) {
        launch(args);
    }

    public static WritableImage getProcessedImage(String path) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        Mat src = Imgcodecs.imread(path);
        Mat dst = new Mat();
        Mat edges = new Mat();
        Mat kernel = new Mat();

        //Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2HSV);
        Imgproc.Canny(src, edges, 45,100);

        return packageImage(edges);
    }

    public static WritableImage packageImage(Mat result) {
        try {
            MatOfByte matOfByte = new MatOfByte();
            Imgcodecs.imencode(".png", result, matOfByte);
            byte[] byteArray = matOfByte.toArray();
            InputStream in = new ByteArrayInputStream(byteArray);
            return SwingFXUtils.toFXImage(ImageIO.read(in), null);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }
}