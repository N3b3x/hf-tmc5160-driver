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
      [ "Key Capabilities", "index.html#autotoc_md35", null ],
      [ "🔀 Chip Compatibility", "index.html#autotoc_md36", null ]
    ] ],
    [ "✨ Features", "index.html#autotoc_md37", [
      [ "🎯 Core Motor Control", "index.html#autotoc_md38", [
        [ "RampControl Subsystem", "index.html#autotoc_md39", null ],
        [ "MotorControl Subsystem", "index.html#autotoc_md40", null ]
      ] ],
      [ "🔍 Diagnostics & Tuning", "index.html#autotoc_md41", [
        [ "Status Subsystem (driver.status)", "index.html#autotoc_md42", null ],
        [ "StallGuard Subsystem (driver.stallGuard)", "index.html#autotoc_md43", null ],
        [ "Tuning Subsystem (driver.tuning)", "index.html#autotoc_md44", null ]
      ] ],
      [ "🏠 Homing & Positioning", "index.html#autotoc_md45", [
        [ "Homing Subsystem (driver.homing)", "index.html#autotoc_md46", null ]
      ] ],
      [ "🔄 Encoder Integration", "index.html#autotoc_md47", [
        [ "Encoder Subsystem", "index.html#autotoc_md48", null ]
      ] ],
      [ "🛡️ Protection Systems", "index.html#autotoc_md49", [
        [ "PowerStage Subsystem (driver.powerStage)", "index.html#autotoc_md50", null ]
      ] ],
      [ "🔗 Multi-Chip Communication", "index.html#autotoc_md51", [
        [ "Communication Subsystem", "index.html#autotoc_md52", null ]
      ] ],
      [ "⚙️ Advanced Features", "index.html#autotoc_md53", null ],
      [ "🏗️ Platform & Architecture", "index.html#autotoc_md54", null ]
    ] ],
    [ "🚀 Quick Start", "index.html#autotoc_md55", null ],
    [ "🔧 Installation", "index.html#autotoc_md56", null ],
    [ "📖 API Reference", "index.html#autotoc_md57", [
      [ "Class Structure & Subsystems", "index.html#autotoc_md58", null ],
      [ "Key Methods by Subsystem", "index.html#autotoc_md59", [
        [ "RampControl Subsystem", "index.html#autotoc_md60", null ],
        [ "Switches Subsystem", "index.html#autotoc_md61", null ],
        [ "Thresholds Subsystem", "index.html#autotoc_md62", null ],
        [ "PowerStage Subsystem", "index.html#autotoc_md63", null ],
        [ "MotorControl Subsystem (driver.motorControl)", "index.html#autotoc_md64", null ],
        [ "StallGuard Subsystem (driver.stallGuard)", "index.html#autotoc_md65", null ],
        [ "Status Subsystem (driver.status)", "index.html#autotoc_md66", null ],
        [ "Tuning Subsystem (driver.tuning)", "index.html#autotoc_md67", null ],
        [ "Homing Subsystem (driver.homing)", "index.html#autotoc_md68", null ],
        [ "Encoder Subsystem (driver.encoder)", "index.html#autotoc_md69", null ],
        [ "PowerStage Subsystem (driver.powerStage)", "index.html#autotoc_md70", null ],
        [ "Thresholds Subsystem (driver.thresholds)", "index.html#autotoc_md71", null ],
        [ "Communication Subsystem (driver.communication)", "index.html#autotoc_md72", null ]
      ] ],
      [ "Multi-Chip Support", "index.html#autotoc_md73", null ],
      [ "Unit Conversion Helpers", "index.html#autotoc_md74", null ]
    ] ],
    [ "📊 Examples", "index.html#autotoc_md75", [
      [ "Comprehensive Test Suite", "index.html#autotoc_md76", null ],
      [ "Motion Control", "index.html#autotoc_md77", null ],
      [ "Tuning & Homing", "index.html#autotoc_md78", null ],
      [ "Multi-Chip Communication", "index.html#autotoc_md79", null ],
      [ "Sensors & Diagnostics", "index.html#autotoc_md80", null ]
    ] ],
    [ "📚 Documentation", "index.html#autotoc_md81", [
      [ "Getting Started", "index.html#autotoc_md82", null ],
      [ "Reference", "index.html#autotoc_md83", null ],
      [ "Advanced Features", "index.html#autotoc_md84", null ]
    ] ],
    [ "🔗 References", "index.html#autotoc_md85", null ],
    [ "🤝 Contributing", "index.html#autotoc_md86", null ],
    [ "📄 License", "index.html#autotoc_md87", null ],
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
"classtmc51x0_1_1TMC51x0MultiNode.html#a751442b2d1d7400ab4450b034d42597e",
"espnow__receiver_8hpp.html#a24fe7b15e7685c89e8468b1fdbcbaf7d",
"internal__ramp__comprehensive__test_8cpp.html#a31a4c053ffd992cb44486a7a9f520ab5",
"namespacePairingMsgType.html#a77b14a12c6912207e5c322c97bfbb40c",
"namespacetmc51x0.html#ac801995f570807ad863a40346b5b7294",
"structPairingRequestPayload.html#a2fdf57a6a1bb279b4ad1e2b5b6d260d1",
"structtmc51x0_1_1DriverConfig.html#a87f4505ddcf4602019dccbf88143519b",
"structtmc51x0_1_1ReferenceSwitchConfig.html",
"structtmc51x0_1_1TMC51x0_1_1Events.html#a6cd548dd8cd411e33456b57f54c917e2",
"structtmc51x0_1_1TMC51x0_1_1Printer.html#a532124eb9225ae250b7bd4b7b7aae620",
"structtmc51x0_1_1TMC51x0_1_1Thresholds.html#aff49a9e83255a88fec0e50283e71a3aa",
"structtmc51x0__test__config_1_1MotorConfig__17HS4401S__Direct.html#a84b95652b50be25c72b9e02c858d906e",
"structtmc51x0__test__config_1_1TestConfig__AppliedMotion__5034.html",
"tmc51x0__types_8hpp.html#a43a3659df2d2d9c81f83cfc72daad756a9ffda00fee81762057c7f5e075196cc0",
"uniontmc51x0_1_1DRV__STATUS__Register.html#a6f7521669355ed7b8c8d4cf802aa9e83",
"uniontmc51x0_1_1OTP__PROG__Register.html#ac6e8979447e2434da3e1046bc6a697e2"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';