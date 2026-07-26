#include "FileDropWatcher.hpp"

#if defined(__linux__)

#include <X11/Xatom.h>
#include <X11/Xlib.h>

#include <cstdlib>
#include <cstring>

namespace p2d::app {

namespace {
Atom internAtom(Display* display, const char* name) { return XInternAtom(display, name, False); }

// text/uri-list is one file:// URI per line (CRLF-terminated per RFC 2483).
// The path portion is percent-encoded (spaces and other special characters
// in filenames), so it needs decoding back to a real filesystem path.
std::vector<std::filesystem::path> parseUriList(const char* data, unsigned long length) {
    std::vector<std::filesystem::path> paths;
    std::string all(data, static_cast<size_t>(length));

    size_t pos = 0;
    while (pos < all.size()) {
        size_t end = all.find('\n', pos);
        if (end == std::string::npos) end = all.size();
        std::string line = all.substr(pos, end - pos);
        pos = end + 1;
        while (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;

        const std::string prefix = "file://";
        if (line.compare(0, prefix.size(), prefix) != 0) continue; // ignore non-local URIs
        std::string encoded = line.substr(prefix.size());

        std::string decoded;
        decoded.reserve(encoded.size());
        for (size_t i = 0; i < encoded.size(); ++i) {
            if (encoded[i] == '%' && i + 2 < encoded.size()) {
                char hex[3] = {encoded[i + 1], encoded[i + 2], '\0'};
                decoded.push_back(static_cast<char>(std::strtol(hex, nullptr, 16)));
                i += 2;
            } else {
                decoded.push_back(encoded[i]);
            }
        }
        paths.emplace_back(decoded);
    }
    return paths;
}
} // namespace

struct FileDropWatcher::Impl {
    Display* display = nullptr;
    ::Window window = 0;
    ::Window sourceWindow = 0; // the dragging application's window, from XdndEnter/Position/Drop
    Atom xdndAware = 0, xdndEnter = 0, xdndPosition = 0, xdndStatus = 0, xdndDrop = 0, xdndFinished = 0,
         xdndSelection = 0, xdndActionCopy = 0, textUriList = 0;
    bool awaitingSelectionNotify = false;
};

FileDropWatcher::FileDropWatcher(sf::WindowHandle windowHandle) {
    impl_ = new Impl();
    impl_->window = static_cast<::Window>(windowHandle);

    // A separate connection from SFML's own -- X11 happily allows more than
    // one client to select events on the same window, so this never
    // competes with or disturbs SFML's event pump.
    impl_->display = XOpenDisplay(nullptr);
    if (!impl_->display) return; // no X server reachable (e.g. this project's own dev sandbox) -- poll() no-ops

    impl_->xdndAware = internAtom(impl_->display, "XdndAware");
    impl_->xdndEnter = internAtom(impl_->display, "XdndEnter");
    impl_->xdndPosition = internAtom(impl_->display, "XdndPosition");
    impl_->xdndStatus = internAtom(impl_->display, "XdndStatus");
    impl_->xdndDrop = internAtom(impl_->display, "XdndDrop");
    impl_->xdndFinished = internAtom(impl_->display, "XdndFinished");
    impl_->xdndSelection = internAtom(impl_->display, "XdndSelection");
    impl_->xdndActionCopy = internAtom(impl_->display, "XdndActionCopy");
    impl_->textUriList = internAtom(impl_->display, "text/uri-list");

    // Advertise XDND protocol version 5 support on this window so a
    // dragging application (e.g. the desktop file manager) knows it's a
    // valid drop target at all.
    long version = 5;
    XChangeProperty(impl_->display, impl_->window, impl_->xdndAware, XA_ATOM, 32, PropModeReplace,
                     reinterpret_cast<unsigned char*>(&version), 1);
    XFlush(impl_->display);
}

FileDropWatcher::~FileDropWatcher() {
    if (impl_) {
        if (impl_->display) XCloseDisplay(impl_->display);
        delete impl_;
    }
}

std::vector<std::filesystem::path> FileDropWatcher::poll() {
    std::vector<std::filesystem::path> dropped;
    if (!impl_->display) return dropped;

    while (XPending(impl_->display) > 0) {
        XEvent event;
        XNextEvent(impl_->display, &event);

        if (event.type == SelectionNotify && impl_->awaitingSelectionNotify) {
            impl_->awaitingSelectionNotify = false;

            Atom actualType;
            int actualFormat;
            unsigned long itemCount = 0, bytesAfter = 0;
            unsigned char* data = nullptr;
            if (XGetWindowProperty(impl_->display, impl_->window, impl_->xdndSelection, 0, 65536, False,
                                    AnyPropertyType, &actualType, &actualFormat, &itemCount, &bytesAfter,
                                    &data) == Success &&
                data) {
                auto paths = parseUriList(reinterpret_cast<const char*>(data), itemCount);
                dropped.insert(dropped.end(), paths.begin(), paths.end());
                XFree(data);
            }

            // Tell the dragging application the drop is complete.
            XClientMessageEvent finished{};
            finished.type = ClientMessage;
            finished.display = impl_->display;
            finished.window = impl_->sourceWindow;
            finished.message_type = impl_->xdndFinished;
            finished.format = 32;
            finished.data.l[0] = static_cast<long>(impl_->window);
            finished.data.l[1] = 1; // accepted
            finished.data.l[2] = static_cast<long>(impl_->xdndActionCopy);
            XSendEvent(impl_->display, impl_->sourceWindow, False, NoEventMask, reinterpret_cast<XEvent*>(&finished));
            XFlush(impl_->display);
            continue;
        }

        if (event.type != ClientMessage) continue;
        Atom messageType = event.xclient.message_type;

        if (messageType == impl_->xdndEnter) {
            impl_->sourceWindow = static_cast<::Window>(event.xclient.data.l[0]);
        } else if (messageType == impl_->xdndPosition) {
            impl_->sourceWindow = static_cast<::Window>(event.xclient.data.l[0]);

            // Accept unconditionally -- this app doesn't distinguish drop
            // zones at the X11 protocol level (any location on the window
            // is a valid drop; importDroppedFiles() below decides what to
            // do with whatever comes in).
            XClientMessageEvent status{};
            status.type = ClientMessage;
            status.display = impl_->display;
            status.window = impl_->sourceWindow;
            status.message_type = impl_->xdndStatus;
            status.format = 32;
            status.data.l[0] = static_cast<long>(impl_->window);
            status.data.l[1] = 1; // accept, and skip further position events for this drag
            status.data.l[2] = 0;
            status.data.l[3] = 0;
            status.data.l[4] = static_cast<long>(impl_->xdndActionCopy);
            XSendEvent(impl_->display, impl_->sourceWindow, False, NoEventMask, reinterpret_cast<XEvent*>(&status));
            XFlush(impl_->display);
        } else if (messageType == impl_->xdndDrop) {
            impl_->sourceWindow = static_cast<::Window>(event.xclient.data.l[0]);
            impl_->awaitingSelectionNotify = true;
            Time timestamp = static_cast<Time>(event.xclient.data.l[2]);
            XConvertSelection(impl_->display, impl_->xdndSelection, impl_->textUriList, impl_->xdndSelection,
                               impl_->window, timestamp);
            XFlush(impl_->display);
        }
    }

    return dropped;
}

} // namespace p2d::app

#else // !__linux__

namespace p2d::app {

struct FileDropWatcher::Impl {};

FileDropWatcher::FileDropWatcher(sf::WindowHandle) {}
FileDropWatcher::~FileDropWatcher() {}
std::vector<std::filesystem::path> FileDropWatcher::poll() { return {}; }

} // namespace p2d::app

#endif
