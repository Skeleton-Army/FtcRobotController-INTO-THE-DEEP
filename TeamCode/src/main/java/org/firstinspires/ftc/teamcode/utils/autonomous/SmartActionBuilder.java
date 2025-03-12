//package org.firstinspires.ftc.teamcode.utils.autonomous;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.utils.actions.RaceAction;
//import org.firstinspires.ftc.teamcode.utils.actions.SleepUntilAction;
//
//import java.lang.reflect.Proxy;
//
//public class SmartActionBuilder {
//    private final TrajectoryActionBuilder builder;
//    private final MecanumDrive drive;
//    private Vector2d lastPosition;
//
//    public SmartActionBuilder(TrajectoryActionBuilder builder, MecanumDrive drive) {
//        this.builder = createTrackingProxy(builder); // Wrap builder with proxy
//        this.drive = drive;
//        this.lastPosition = drive.pose.position; // Start position
//    }
//
//    private TrajectoryActionBuilder createTrackingProxy(TrajectoryActionBuilder originalBuilder) {
//        return (TrajectoryActionBuilder) Proxy.newProxyInstance(
//                originalBuilder.getClass().getClassLoader(),
//                new Class<?>[]{TrajectoryActionBuilder.class},
//                (proxy, method, args) -> {
//                    // If the method is `build()`, call build on SmartActionBuilder instead
//                    if (method.getName().equals("build")) {
//                        return SmartActionBuilder.this.build();
//                    }
//
//                    // If the method has Vector2d or Pose2d as an argument, update lastPosition
//                    if (args != null) {
//                        for (Object arg : args) {
//                            if (arg instanceof Vector2d) {
//                                lastPosition = (Vector2d) arg;
//                                break;
//                            } else if (arg instanceof Pose2d) {
//                                lastPosition = ((Pose2d) arg).position;
//                                break;
//                            }
//                        }
//                    }
//
//                    // Otherwise, invoke the method on the original builder
//                    return method.invoke(originalBuilder, args);
//                }
//        );
//    }
//
//    public Action build() {
//        Action trajectoryAction = builder.build();
//
//        return new RaceAction(
//                trajectoryAction,
//                new SleepUntilAction(() ->
//                        Math.abs(drive.pose.position.x - lastPosition.x) < 0.2 &&
//                        Math.abs(drive.pose.position.y - lastPosition.y) < 0.2
//                )
//        );
//    }
//
//    public TrajectoryActionBuilder getBuilder() {
//        return builder;
//    }
//}
