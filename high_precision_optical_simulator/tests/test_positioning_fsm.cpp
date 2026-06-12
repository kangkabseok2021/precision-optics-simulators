#include <gtest/gtest.h>
#include "optical/PositioningFsm.h"
#include <string>
#include <vector>

// Minimal mock — configurable pass/fail for init() and selfTest().
class MockDevice : public IDevice {
public:
    bool init_ok  = true;
    bool test_ok  = true;
    bool shutdown_called = false;

    bool  init()     override { return init_ok; }
    bool  selfTest() override { return test_ok; }
    void  shutdown() override { shutdown_called = true; }
    const char* name() const override { return "MockDevice"; }
};

TEST(FsmTest, StartsInIdle) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    EXPECT_EQ(fsm.state(), PosState::IDLE);
}

TEST(FsmTest, StartTransitionsToInitialising) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    EXPECT_TRUE(fsm.start());
    EXPECT_EQ(fsm.state(), PosState::INITIALISING);
}

TEST(FsmTest, CalibrateReachesReady) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    fsm.start();
    EXPECT_TRUE(fsm.calibrate());
    EXPECT_EQ(fsm.state(), PosState::READY);
}

TEST(FsmTest, ActivateFromReady) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    fsm.start();
    fsm.calibrate();
    fsm.activate();
    EXPECT_EQ(fsm.state(), PosState::ACTIVE);
}

TEST(FsmTest, PauseBackToReady) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    fsm.start();
    fsm.calibrate();
    fsm.activate();
    fsm.pause();
    EXPECT_EQ(fsm.state(), PosState::READY);
}

TEST(FsmTest, FaultFromActiveGoesFault) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    fsm.start();
    fsm.calibrate();
    fsm.activate();
    fsm.fault("overpressure");
    EXPECT_EQ(fsm.state(), PosState::FAULT);
}

TEST(FsmTest, FaultCapturesReason) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    fsm.fault("test_reason");
    EXPECT_EQ(fsm.lastError(), "test_reason");
}

TEST(FsmTest, FaultCallsShutdownOnDevices) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    fsm.fault("any");
    EXPECT_TRUE(dev.shutdown_called);
}

TEST(FsmTest, ResetFromFaultToIdle) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    fsm.fault("any");
    EXPECT_TRUE(fsm.reset());
    EXPECT_EQ(fsm.state(), PosState::IDLE);
    EXPECT_TRUE(fsm.lastError().empty());
}

TEST(FsmTest, SelfTestFailureGoesFault) {
    MockDevice dev;
    dev.test_ok = false;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    fsm.start();
    EXPECT_FALSE(fsm.calibrate());
    EXPECT_EQ(fsm.state(), PosState::FAULT);
    EXPECT_NE(fsm.lastError().find("selfTest"), std::string::npos);
}

TEST(FsmTest, StartFromFaultReturnsFalse) {
    MockDevice dev;
    std::vector<IDevice*> devs{&dev};
    PositioningFsm fsm(devs);
    fsm.fault("any");
    EXPECT_FALSE(fsm.start());
    EXPECT_EQ(fsm.state(), PosState::FAULT);
}
