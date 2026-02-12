/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "HF-TMC51x0 Driver (TMC5130 & TMC5160)", "index.html", [
    [ "📚 Table of Contents", "index.html#autotoc_md32", null ],
    [ "📦 Overview", "index.html#autotoc_md33", [
      [ "Architecture & Design", "index.html#autotoc_md34", null ],
      [ "Key Capabilities", "index.html#autotoc_md35", null ]
    ] ],
    [ "✨ Features", "index.html#autotoc_md36", [
      [ "🎯 Core Motor Control", "index.html#autotoc_md37", [
        [ "RampControl Subsystem", "index.html#autotoc_md38", null ],
        [ "MotorControl Subsystem", "index.html#autotoc_md39", null ]
      ] ],
      [ "🔍 Diagnostics & Tuning", "index.html#autotoc_md40", [
        [ "Status Subsystem (driver.status)", "index.html#autotoc_md41", null ],
        [ "StallGuard Subsystem (driver.stallGuard)", "index.html#autotoc_md42", null ],
        [ "Tuning Subsystem (driver.tuning)", "index.html#autotoc_md43", null ]
      ] ],
      [ "🏠 Homing & Positioning", "index.html#autotoc_md44", [
        [ "Homing Subsystem (driver.homing)", "index.html#autotoc_md45", null ]
      ] ],
      [ "🔄 Encoder Integration", "index.html#autotoc_md46", [
        [ "Encoder Subsystem", "index.html#autotoc_md47", null ]
      ] ],
      [ "🛡️ Protection Systems", "index.html#autotoc_md48", [
        [ "PowerStage Subsystem (driver.powerStage)", "index.html#autotoc_md49", null ]
      ] ],
      [ "🔗 Multi-Chip Communication", "index.html#autotoc_md50", [
        [ "Communication Subsystem", "index.html#autotoc_md51", null ]
      ] ],
      [ "⚙️ Advanced Features", "index.html#autotoc_md52", null ],
      [ "🏗️ Platform & Architecture", "index.html#autotoc_md53", null ]
    ] ],
    [ "🚀 Quick Start", "index.html#autotoc_md54", [
      [ "Single Motor Setup", "index.html#autotoc_md55", null ],
      [ "Multi-Motor Daisy Chain Setup", "index.html#autotoc_md56", null ],
      [ "Using Physical Units", "index.html#autotoc_md57", null ],
      [ "Error Handling – the Result<T> Pattern", "index.html#autotoc_md58", null ]
    ] ],
    [ "🔧 Installation", "index.html#autotoc_md59", null ],
    [ "API Reference", "index.html#autotoc_md60", [
      [ "Class Structure & Subsystems", "index.html#autotoc_md61", null ],
      [ "Key Methods by Subsystem", "index.html#autotoc_md62", [
        [ "RampControl Subsystem", "index.html#autotoc_md63", null ],
        [ "Switches Subsystem", "index.html#autotoc_md64", null ],
        [ "Thresholds Subsystem", "index.html#autotoc_md65", null ],
        [ "PowerStage Subsystem", "index.html#autotoc_md66", null ],
        [ "MotorControl Subsystem (driver.motorControl)", "index.html#autotoc_md67", null ],
        [ "StallGuard Subsystem (driver.stallGuard)", "index.html#autotoc_md68", null ],
        [ "Status Subsystem (driver.status)", "index.html#autotoc_md69", null ],
        [ "Tuning Subsystem (driver.tuning)", "index.html#autotoc_md70", null ],
        [ "Homing Subsystem (driver.homing)", "index.html#autotoc_md71", null ],
        [ "Encoder Subsystem (driver.encoder)", "index.html#autotoc_md72", null ],
        [ "PowerStage Subsystem (driver.powerStage)", "index.html#autotoc_md73", null ],
        [ "Thresholds Subsystem (driver.thresholds)", "index.html#autotoc_md74", null ],
        [ "Communication Subsystem (driver.communication)", "index.html#autotoc_md75", null ]
      ] ],
      [ "Multi-Chip Support", "index.html#autotoc_md76", null ],
      [ "Unit Conversion Helpers", "index.html#autotoc_md77", null ]
    ] ],
    [ "📊 Examples", "index.html#autotoc_md78", [
      [ "Comprehensive Test Suite", "index.html#autotoc_md79", null ],
      [ "Motion Control", "index.html#autotoc_md80", null ],
      [ "Tuning & Homing", "index.html#autotoc_md81", null ],
      [ "Multi-Chip Communication", "index.html#autotoc_md82", null ],
      [ "Sensors & Diagnostics", "index.html#autotoc_md83", null ]
    ] ],
    [ "Documentation", "index.html#autotoc_md84", [
      [ "Getting Started", "index.html#autotoc_md85", null ],
      [ "Reference", "index.html#autotoc_md86", null ],
      [ "Advanced Features", "index.html#autotoc_md87", null ]
    ] ],
    [ "🤝 Contributing", "index.html#autotoc_md88", null ],
    [ "📄 License", "index.html#autotoc_md89", null ],
    [ "Deprecated List", "deprecated.html", null ],
    [ "Topics", "topics.html", "topics" ],
    [ "Namespaces", "namespaces.html", [
      [ "Namespace List", "namespaces.html", "namespaces_dup" ],
      [ "Namespace Members", "namespacemembers.html", [
        [ "All", "namespacemembers.html", "namespacemembers_dup" ],
        [ "Functions", "namespacemembers_func.html", null ],
        [ "Variables", "namespacemembers_vars.html", null ],
        [ "Typedefs", "namespacemembers_type.html", null ],
        [ "Enumerations", "namespacemembers_enum.html", null ]
      ] ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", "functions_func" ],
        [ "Variables", "functions_vars.html", "functions_vars" ],
        [ "Typedefs", "functions_type.html", null ],
        [ "Enumerations", "functions_enum.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", "globals_dup" ],
        [ "Functions", "globals_func.html", null ],
        [ "Variables", "globals_vars.html", null ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"TestFramework_8h.html",
"classFatigueTest_1_1FatigueTestMotion.html#acc783c726d21ac36fff70a31ad108864",
"classtmc51x0_1_1Result.html#ab74d4fb35bd2933480a2b417f02fd59d",
"classtmc51x0_1_1TMC51x0MultiNode.html#ac3198576e0c6cec08b6cd3423c42d8ca",
"espnow__security_8hpp.html#a5b51642f162ee5a7535b3090a8967aeb",
"internal__ramp__comprehensive__test_8cpp.html#a6c66dbed0857e4ad6dfacbe0a5ba9a4d",
"namespaceanonymous__namespace_02espnow__peer__store_8cpp_03.html#a354afd1c551506965f65e541f88bcb6b",
"namespacetmc51x0.html#af55e97adffb8cd4694d00b13ebc75db9a21994d6177b29e1128b2d7f0f8342057",
"structRawMsg.html#a09618c9e79f7582ab247921e1e47cf19",
"structtmc51x0_1_1ExternalClockConfig.html",
"structtmc51x0_1_1SpiCommand.html#afc505ae47fa8420639839668c74409ac",
"structtmc51x0_1_1TMC51x0_1_1Homing.html#a5798952825e02afe3755b1b9a55976cb",
"structtmc51x0_1_1TMC51x0_1_1RampControl.html#a56fb3e0ece01a796026b947a04b782df",
"structtmc51x0_1_1TMC51x0_1_1WriteOnlyRegisters.html#a41cdf60a8ef6e2d4f3fd747e9487bb54",
"structtmc51x0__test__config_1_1MotorConfig__AppliedMotion__5034__369.html",
"structtmc51x0__test__config_1_1TestRigConfig_3_01TestRigType_1_1TEST__RIG__CORE__DRIVER_01_4.html#a1651c81eb8ef7cb4fe09ae8658d9e0fb",
"tmc51x0__types_8hpp.html#a8cb45f50ef60b0895ae7c456412b9f57",
"uniontmc51x0_1_1ENCMODE__Register.html#a36a87f34bbbfef0e575d99d7f4d7e30f",
"uniontmc51x0_1_1PWMCONF__Register.html#af1f1c792c205de7cd34dc29a321bc6b3"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';