package org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer;

import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.Indexer;

public class MotorIndexerBlank implements MotorIndexer {
    @Override
    public double getPositionError() {
        return 0;
    }

    @Override
    public double getOutputPower() {
        return 0;
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public int getKSFlag() {
        return 0;
    }

    @Override
    public double getRd() {
        return 0;
    }

    @Override
    public void manualOverride(int direction) {

    }

    @Override
    public void unload() {

    }

    @Override
    public void resetEncoder() {

    }

    @Override
    public void goTo(Indexer.Index index) {

    }

    @Override
    public void next() {

    }

    @Override
    public void prepIntake() {

    }

    @Override
    public void intake() {

    }

    @Override
    public double getEncoder() {
        return 0;
    }

    @Override
    public Indexer.Index getIndex() {
        return null;
    }

    @Override
    public double getRelative() {
        return 0;
    }

    @Override
    public void update() {

    }
}
