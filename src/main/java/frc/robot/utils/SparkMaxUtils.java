package frc.robot.utils;

import com.revrobotics.REVLibError;

public final class SparkMaxUtils {
    public static double cleanSparkMaxValue(double lastValue, double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)
                || (Math.abs(value) < 1.0e-4 && Math.abs(lastValue) > 60.0)) {
            return lastValue;
        } 
        return value;
    }
    public static String REVErrorToString(REVLibError error){
        switch (error) {
            case kOk: return "kOk";
            case kError: return "kError";
            case kTimeout: return "kTimeout";
            case kNotImplemented: return "kNotImplemented";
            case kHALError: return "kHALError";
            case kCantFindFirmware: return "kCantFindFirmware";
            case kFirmwareTooOld: return "kFirmwareTooOld";
            case kFirmwareTooNew: return "kFirmwareTooNew";
            case kParamInvalidID: return "kParamInvalidID";
            case kParamMismatchType: return "kParamMismatchType";
            case kParamAccessMode: return "kParamAccessMode";
            case kParamInvalid: return "kParamInvalid";
            case kParamNotImplementedDeprecated: return "kParamNotImplementedDeprecated";
            case kFollowConfigMismatch: return "kFollowConfigMismatch";
            case kInvalid: return "kInvalid";
            case kSetpointOutOfRange: return "kSetpointOutOfRange";
            case kUnknown: return "kUnknown";
            case kCANDisconnected: return "kCANDisconnected";
            case kDuplicateCANId: return "kDuplicateCANId";
            case kInvalidCANId: return "kInvalidCANId";
            case kSparkMaxDataPortAlreadyConfiguredDifferently: return "kSparkMaxDataPortAlreadyConfiguredDifferently";
            default: return "Unknown";
        }
    }
}