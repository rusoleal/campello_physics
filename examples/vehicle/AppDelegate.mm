#import "AppDelegate.h"
#import "ViewController.h"

@implementation AppDelegate {
    NSWindow* _window;
}

- (void)applicationDidFinishLaunching:(NSNotification*)notification {
    NSRect frame = NSMakeRect(0, 0, 1280, 720);
    _window = [[NSWindow alloc]
        initWithContentRect:frame
        styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                   NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable)
        backing:NSBackingStoreBuffered
        defer:NO];
    _window.title = @"Vehicle — campello_physics";

    ViewController* vc = [[ViewController alloc] init];
    _window.contentViewController = vc;
    [_window center];
    [_window makeKeyAndOrderFront:nil];
    [_window makeFirstResponder:vc];
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)sender {
    return YES;
}

@end
