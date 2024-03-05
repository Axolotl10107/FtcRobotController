package org.firstinspires.ftc.teamcode.fy23.fakestuff;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class MockElapsedTime extends ElapsedTime {

    private long nanos = 0;

    public MockElapsedTime() {
        super();
    }

    public MockElapsedTime(long startTime) {
        super(startTime);
    }

    public MockElapsedTime(Resolution resolution) {
        super(resolution);
    }

    // all other methods of ElapsedTime call this, so they'll still work with our controlled time
    @Override
    protected long nsNow() {
        return nanos;
    }

    public void setNanos(long nanos) {
        this.nanos = nanos;
    }

    public void advanceNanos(long nanos) {
        setNanos(nsNow() + nanos);
    }

    public void setTime(long time, TimeUnit unit) {
        setNanos(TimeUnit.NANOSECONDS.convert(time, unit));
        // converts time in unit to nanoseconds
    }

    public void advanceTime(long time, TimeUnit unit) {
        setNanos(nsNow() + TimeUnit.NANOSECONDS.convert(time, unit));
    }

}
