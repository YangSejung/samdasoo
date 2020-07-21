package com.taehyun.pointcloud.Utils;

import com.google.ar.core.Frame;

public class SingleTonClass {
    private static final SingleTonClass ourInstance = new SingleTonClass();

    public static SingleTonClass getInstance() {
        return ourInstance;
    }

    private SingleTonClass() {
    }

    public Frame frame;
}
