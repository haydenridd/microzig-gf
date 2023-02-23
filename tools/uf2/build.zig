const std = @import("std");

pub fn build(b: *std.build.Builder) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const lib = b.addStaticLibrary(.{
        .name = "uf2",
        .root_source_file = .{ .path = "src/main.zig" },
        .target = target,
        .optimize = optimize,
    });
    lib.install();

    const main_tests = b.addTest(.{
        .root_source_file = .{ .path = "src/main.zig" },
    });

    const test_step = b.step("test", "Run library tests");
    test_step.dependOn(&main_tests.step);

    const gen = b.addExecutable(.{
        .name = "gen",
        .root_source_file = .{ .path = "src/gen.zig" },
    });
    const gen_run_step = gen.run();
    const gen_step = b.step("gen", "Generate family id enum");
    gen_step.dependOn(&gen_run_step.step);

    const exe = b.addExecutable(.{
        .name = "example",
        .root_source_file = .{ .path = "src/example.zig" },
    });
    exe.install();
}