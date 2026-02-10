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
"espnow__peer__store_8cpp.html#a5a85b9c772bbeb480b209a3e6ea92b4c",
"globals.html",
"main_8cpp.html#a25a95fc0d7fa9ed58edd599d795ecd86",
"namespacetmc51x0.html#a20af559eb85dbac784b3a4cb644cf63bad3fe043af498c0c3096222212fbeef50",
"namespacetmc51x0__test__config.html#a7bc2612c019048baa387dc973d3c2365",
"structstd_1_1tuple__size_3_01tmc51x0_1_1Result_3_01void_01_4_01_4.html",
"structtmc51x0_1_1InputStatus.html#a36deada4793988b09e7f83cad2eae02a",
"structtmc51x0_1_1StallGuardTuningResult.html#a569d0acfacd8af0f787ebf7c3f45e1c5",
"structtmc51x0_1_1TMC51x0_1_1Printer.html#acff7f8cc159716b3a4585ec1bb2f384f",
"structtmc51x0__test__config_1_1MotorConfig__17HS4401S.html#a1c8f60ed2391c01a2b7cf5f518a80238",
"structtmc51x0__test__config_1_1PlatformConfig__FatigueTestRig_1_1Encoder.html#ae14a4105feee32e5a1cc248f9beb47e0",
"tmc51x0__motor__calc_8hpp.html#a735799ad1db46015a3121adbf1cec9ac",
"uart__multi__node__comprehensive__test_8cpp.html#ae310b1c5cb8c9a802578592d44b66126",
"uniontmc51x0_1_1IHOLD__IRUN__Register.html#a32a5682e4e399b1f3b6334e56604ddcc",
"uniontmc51x0_1_1SW__MODE__Register.html#a2a5ecb7ecab609fbe440df094dcf68c1"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';