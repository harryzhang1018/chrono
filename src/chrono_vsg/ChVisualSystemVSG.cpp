// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// Screen capture code from https://github.com/vsg-dev/vsgExamples.git
//
// =============================================================================
// Radu Serban, Rainer Gericke
// =============================================================================

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>
#include "chrono_vsg/tools/createSkybox.h"
#include "chrono_vsg/tools/createQuad.h"
#include "ChVisualSystemVSG.h"
#include "chrono_thirdparty/stb/stb_image_write.h"
#include "chrono_thirdparty/stb/stb_image_resize.h"
#include <algorithm>
#include <string>
#include <cstddef>
#include <cctype>

namespace chrono {
namespace vsg3d {

using namespace std;

class AppKeyboardHandler : public vsg::Inherit<vsg::Visitor, AppKeyboardHandler> {
  public:
    AppKeyboardHandler(vsg::Viewer* viewer) : m_viewer(viewer) {}

    void SetParams(vsg::ref_ptr<ChVisualSystemVSG::StateParams> params, ChVisualSystemVSG* appPtr) {
        _params = params;
        m_appPtr = appPtr;
    }

    void apply(vsg::KeyPressEvent& keyPress) override {
        if (keyPress.keyBase == 'm' || keyPress.keyModified == 'm') {
            // toggle graphical menu
            _params->showGui = !_params->showGui;
        }
        if (keyPress.keyBase == 't' || keyPress.keyModified == 't') {
            // terminate process
            m_appPtr->Quit();
        }
    }

  private:
    vsg::observer_ptr<vsg::Viewer> m_viewer;
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> _params;
    ChVisualSystemVSG* m_appPtr;
};

class GuiComponent {
  public:
    GuiComponent(vsg::ref_ptr<ChVisualSystemVSG::StateParams> params, ChVisualSystemVSG* appPtr)
        : _params(params), m_appPtr(appPtr) {}

    // Example here taken from the Dear imgui comments (mostly)
    bool operator()() {
        bool visibleComponents = false;
        ImGuiIO& io = ImGui::GetIO();
#ifdef __APPLE__
        io.FontGlobalScale = 2.0;
#else
        io.FontGlobalScale = 1.0;
#endif

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
        if (_params->showGui) {
            ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
            ImGui::Begin("App:");  // Create a window called "Hello, world!" and append into it.

            if (ImGui::Button(
                    "Quit"))  // Buttons return true when clicked (most widgets return true when edited/activated)
                m_appPtr->Quit();

            ImGui::End();
            visibleComponents = true;
        }

        return visibleComponents;
    }

  private:
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> _params;
    ChVisualSystemVSG* m_appPtr;
};

ChVisualSystemVSG::ChVisualSystemVSG() {
    m_options = vsg::Options::create();
    m_options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    m_options->paths.push_back(GetChronoDataPath());
    m_options->objectCache = vsg::ObjectCache::create();
    // add vsgXchange's support for reading and writing 3rd party file formats
    m_options->add(vsgXchange::all::create());
    m_options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
}

void ChVisualSystemVSG::SetCameraVertical(CameraVerticalDir vert) {
    if (!m_initialized) {
        m_yup = (vert == CameraVerticalDir::Y);
        if (m_yup) {
            m_up_vector = vsg::dvec3(0, 1, 0);
        } else {
            m_up_vector = vsg::dvec3(0, 0, 1);
        }
    } else {
        cout << "SetCameraVertical() cannot be used after calling Initialize()!" << endl;
    }
}

ChVisualSystemVSG::~ChVisualSystemVSG() {}

void ChVisualSystemVSG::SetClearColor(ChColor cc) {
    if (!m_initialized) {
        m_bg_color = cc;
    } else {
        cout << "SetClearColor() can not be used after calling Initialize()!" << endl;
    }
}

void ChVisualSystemVSG::Initialize() {
    m_windowTraits = vsg::WindowTraits::create();
    m_windowTraits->windowTitle = m_windowTitle;
    m_windowTraits->width = m_windowWidth;
    m_windowTraits->height = m_windowHeight;
    m_windowTraits->x = m_windowPosX;
    m_windowTraits->y = m_windowPosY;
    m_windowTraits->deviceExtensionNames = {VK_KHR_MULTIVIEW_EXTENSION_NAME, VK_KHR_MAINTENANCE2_EXTENSION_NAME,
                                            VK_KHR_CREATE_RENDERPASS_2_EXTENSION_NAME,
                                            VK_KHR_DEPTH_STENCIL_RESOLVE_EXTENSION_NAME};

    // enable transfer from the colour and depth buffer images
    m_windowTraits->swapchainPreferences.imageUsage =
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    m_windowTraits->depthImageUsage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

    // create the viewer and assign window(s) to it
    m_viewer = vsg::Viewer::create();

    m_window = vsg::Window::create(m_windowTraits);
    if (!m_window) {
        std::cout << "Could not create window." << std::endl;
        return;
    }

    auto device = m_window->getOrCreateDevice();
    if (!device) {
        std::cout << "Could not create device." << std::endl;
        return;
    }
    m_viewer->addWindow(m_window);

    m_window->clearColor() = VkClearColorValue{{m_bg_color.R, m_bg_color.G, m_bg_color.B, m_bg_color.A}};

    // holds whole 3d stuff
    m_scenegraph = vsg::Group::create();
    if (m_use_skybox) {
        // build node from cubemap texture file
        if (!m_skyboxFilename.empty()) {
            if (auto node = createSkybox(m_skyboxFilename, m_options, m_yup); node) {
                m_scenegraph->addChild(node);
            } else {
                cout << "Couldn't load " << m_skyboxFilename << endl;
            }
        }
    }
    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color.set(1.0, 1.0, 1.0);
    ambientLight->intensity = 0.1;

    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "head light";
    directionalLight->color.set(1.0, 1.0, 1.0);
    directionalLight->intensity = m_light_intensity;
    if(m_yup) {
        directionalLight->direction.set(-cos(m_elevation)*cos(m_acimut), -sin(m_elevation), -cos(m_elevation)*sin(m_acimut));
    } else {
        directionalLight->direction.set(-cos(m_elevation)*cos(m_acimut), -cos(m_elevation)*sin(m_acimut), -sin(m_elevation));
    }

    auto absoluteTransform = vsg::AbsoluteTransform::create();
    absoluteTransform->addChild(ambientLight);
    absoluteTransform->addChild(directionalLight);

    m_scenegraph->addChild(absoluteTransform);

    // compute the bounds of the scene graph to help position camera
    vsg::ComputeBounds computeBounds;
    m_scenegraph->accept(computeBounds);
    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;

    // These are set statically because the geometry in the class is expanded in the shader
    double nearFarRatio = 0.01;

    // set up the camera automatically
    m_center_point = centre;
    if (m_yup) {
        m_eye_point = centre + vsg::dvec3(-radius * 3.5, 0.0, 0.0);
    } else {
        m_eye_point = centre + vsg::dvec3(0.0, -radius * 3.5, 0.0);
    }
    m_lookAt = vsg::LookAt::create(m_eye_point, m_center_point, m_up_vector);
    m_perspective = vsg::Perspective::create(
        30.0, static_cast<double>(m_window->extent2D().width) / static_cast<double>(m_window->extent2D().height),
        nearFarRatio * radius, radius * 400.5);

    m_camera = vsg::Camera::create(m_perspective, m_lookAt, vsg::ViewportState::create(m_window->extent2D()));

    // The commandGraph will contain a 2 stage renderGraph 1) 3D scene 2) ImGui (by default also includes clear depth
    // buffers)
    m_commandGraph = vsg::CommandGraph::create(m_window);
    m_renderGraph = vsg::RenderGraph::create(m_window);

    // create the normal 3D view of the scene
    m_renderGraph->addChild(vsg::View::create(m_camera, m_scenegraph));


    // Imgui graphical menu handler
    m_renderGraph->addChild(vsgImGui::RenderImGui::create(m_window, GuiComponent(m_params, this)));
    m_commandGraph->addChild(m_renderGraph);

    // Add the ImGui event handler first to handle events early
    m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());

    // add keyboard handler
    auto kbHandler = AppKeyboardHandler::create(m_viewer);
    kbHandler->SetParams(m_params, this);
    m_viewer->addEventHandler(kbHandler);

    // add close handler to respond the close window button and pressing escape
    m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));
    m_viewer->addEventHandler(vsg::Trackball::create(m_camera));

    m_viewer->assignRecordAndSubmitTaskAndPresentation({m_commandGraph});

    m_viewer->compile();

    BindAll();

    m_initialized = true;
}

void ChVisualSystemVSG::Render() {
    m_viewer->handleEvents();
    m_viewer->update();
    m_viewer->recordAndSubmit();
    if (m_params->do_image_capture)
        export_image();
    m_viewer->present();
}

bool ChVisualSystemVSG::Run() {
    return m_viewer->advanceToNextFrame();
}

void ChVisualSystemVSG::Quit() {
    m_viewer->close();
}

void ChVisualSystemVSG::SetWindowSize(const ChVector2<int>& win_size) {
    if (!m_initialized) {
        m_windowWidth = win_size[0];
        m_windowHeight = win_size[1];
    } else {
        cout << "SetWindowSize() cannot be used after initializing!" << endl;
    }
}

void ChVisualSystemVSG::SetWindowPosition(const ChVector2<int>& win_pos) {
    if (!m_initialized) {
        m_windowPosX = win_pos[0];
        m_windowPosY = win_pos[1];
    } else {
        cout << "SetWindowPosition() cannot be used after initializing!" << endl;
    }
}

void ChVisualSystemVSG::SetWindowTitle(const std::string& win_title) {
    if (!m_initialized) {
        m_windowTitle = string(win_title);
    } else {
        cout << "SetWindowTitle() cannot be used after initializing!" << endl;
    }
}

void ChVisualSystemVSG::WriteImageToFile(const string& filename) {
#ifdef WIN32
    cout << "Actually there is a serious bug in VSG preventing Windows programs from writing screenshots!" << endl;
#else
    m_imageFilename = string(filename);
    m_params->do_image_capture = true;
#endif
}

void ChVisualSystemVSG::export_image() {
    m_params->do_image_capture = false;
    auto width = m_window->extent2D().width;
    auto height = m_window->extent2D().height;

    auto device = m_window->getDevice();
    auto physicalDevice = m_window->getPhysicalDevice();
    auto swapchain = m_window->getSwapchain();

    // get the colour buffer image of the previous rendered frame as the current frame hasn't been rendered yet.  The 1
    // in window->imageIndex(1) means image from 1 frame ago.
    auto sourceImage = m_window->imageView(m_window->imageIndex(1))->image;

    VkFormat sourceImageFormat = swapchain->getImageFormat();
    VkFormat targetImageFormat = sourceImageFormat;

    //
    // 1) Check to see of Blit is supported.
    //
    VkFormatProperties srcFormatProperties;
    vkGetPhysicalDeviceFormatProperties(*(physicalDevice), sourceImageFormat, &srcFormatProperties);

    VkFormatProperties destFormatProperties;
    vkGetPhysicalDeviceFormatProperties(*(physicalDevice), VK_FORMAT_R8G8B8A8_UNORM, &destFormatProperties);

    bool supportsBlit = ((srcFormatProperties.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_SRC_BIT) != 0) &&
                        ((destFormatProperties.linearTilingFeatures & VK_FORMAT_FEATURE_BLIT_DST_BIT) != 0);

    if (supportsBlit) {
        // we can automatically convert the image format when blit, so take advantage of it to ensure RGBA
        targetImageFormat = VK_FORMAT_R8G8B8A8_UNORM;
    }

    // std::cout<<"supportsBlit = "<<supportsBlit<<std::endl;

    //
    // 2) create image to write to
    //
    auto destinationImage = vsg::Image::create();
    destinationImage->imageType = VK_IMAGE_TYPE_2D;
    destinationImage->format = targetImageFormat;
    destinationImage->extent.width = width;
    destinationImage->extent.height = height;
    destinationImage->extent.depth = 1;
    destinationImage->arrayLayers = 1;
    destinationImage->mipLevels = 1;
    destinationImage->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    destinationImage->samples = VK_SAMPLE_COUNT_1_BIT;
    destinationImage->tiling = VK_IMAGE_TILING_LINEAR;
    destinationImage->usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;

    destinationImage->compile(device);

    auto deviceMemory =
        vsg::DeviceMemory::create(device, destinationImage->getMemoryRequirements(device->deviceID),
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    destinationImage->bind(deviceMemory, 0);

    //
    // 3) create command buffer and submit to graphics queue
    //
    auto commands = vsg::Commands::create();

    // 3.a) transition destinationImage to transfer destination initialLayout
    auto transitionDestinationImageToDestinationLayoutBarrier = vsg::ImageMemoryBarrier::create(
        0,                                                              // srcAccessMask
        VK_ACCESS_TRANSFER_WRITE_BIT,                                   // dstAccessMask
        VK_IMAGE_LAYOUT_UNDEFINED,                                      // oldLayout
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,                           // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        destinationImage,                                               // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    // 3.b) transition swapChainImage from present to transfer source initialLayout
    auto transitionSourceImageToTransferSourceLayoutBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_MEMORY_READ_BIT,                                      // srcAccessMask
        VK_ACCESS_TRANSFER_READ_BIT,                                    // dstAccessMask
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,                                // oldLayout
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                           // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        sourceImage,                                                    // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    auto cmd_transitionForTransferBarrier =
        vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT,                        // srcStageMask
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,                        // dstStageMask
                                     0,                                                     // dependencyFlags
                                     transitionDestinationImageToDestinationLayoutBarrier,  // barrier
                                     transitionSourceImageToTransferSourceLayoutBarrier     // barrier
        );

    commands->addChild(cmd_transitionForTransferBarrier);

    if (supportsBlit) {
        // 3.c.1) if blit using VkCmdBliImage
        VkImageBlit region{};
        region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.srcSubresource.layerCount = 1;
        region.srcOffsets[0] = VkOffset3D{0, 0, 0};
        region.srcOffsets[1] = VkOffset3D{static_cast<int32_t>(width), static_cast<int32_t>(height), 0};
        region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.dstSubresource.layerCount = 1;
        region.dstOffsets[0] = VkOffset3D{0, 0, 0};
        region.dstOffsets[1] = VkOffset3D{static_cast<int32_t>(width), static_cast<int32_t>(height), 0};

        auto blitImage = vsg::BlitImage::create();
        blitImage->srcImage = sourceImage;
        blitImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        blitImage->dstImage = destinationImage;
        blitImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        blitImage->regions.push_back(region);
        blitImage->filter = VK_FILTER_NEAREST;

        commands->addChild(blitImage);
    } else {
        // 3.c.2) else use VkVmdCopyImage

        VkImageCopy region{};
        region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.srcSubresource.layerCount = 1;
        region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.dstSubresource.layerCount = 1;
        region.extent.width = width;
        region.extent.height = height;
        region.extent.depth = 1;

        auto copyImage = vsg::CopyImage::create();
        copyImage->srcImage = sourceImage;
        copyImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        copyImage->dstImage = destinationImage;
        copyImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        copyImage->regions.push_back(region);

        commands->addChild(copyImage);
    }

    // 3.d) transition destinate image from transfer destination layout to general layout to enable mapping to image
    // DeviceMemory
    auto transitionDestinationImageToMemoryReadBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_TRANSFER_WRITE_BIT,                                   // srcAccessMask
        VK_ACCESS_MEMORY_READ_BIT,                                      // dstAccessMask
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,                           // oldLayout
        VK_IMAGE_LAYOUT_GENERAL,                                        // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        destinationImage,                                               // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    // 3.e) transition swap chain image back to present
    auto transitionSourceImageBackToPresentBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_TRANSFER_READ_BIT,                                    // srcAccessMask
        VK_ACCESS_MEMORY_READ_BIT,                                      // dstAccessMask
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                           // oldLayout
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,                                // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        sourceImage,                                                    // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    auto cmd_transitionFromTransferBarrier =
        vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT,                 // srcStageMask
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,                 // dstStageMask
                                     0,                                              // dependencyFlags
                                     transitionDestinationImageToMemoryReadBarrier,  // barrier
                                     transitionSourceImageBackToPresentBarrier       // barrier
        );

    commands->addChild(cmd_transitionFromTransferBarrier);

    auto fence = vsg::Fence::create(device);
    auto queueFamilyIndex = physicalDevice->getQueueFamily(VK_QUEUE_GRAPHICS_BIT);
    auto commandPool = vsg::CommandPool::create(device, queueFamilyIndex);
    auto queue = device->getQueue(queueFamilyIndex);

    vsg::submitCommandsToQueue(device, commandPool, fence, 100000000000, queue,
                               [&](vsg::CommandBuffer& commandBuffer) { commands->record(commandBuffer); });

    //
    // 4) map image and copy
    //
    VkImageSubresource subResource{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0};
    VkSubresourceLayout subResourceLayout;
    vkGetImageSubresourceLayout(*device, destinationImage->vk(device->deviceID), &subResource, &subResourceLayout);

    // Map the buffer memory and assign as a vec4Array2D that will automatically unmap itself on destruction.
    auto imageData = vsg::MappedData<vsg::ubvec4Array2D>::create(deviceMemory, subResourceLayout.offset, 0,
                                                                 vsg::Data::Layout{targetImageFormat}, width,
                                                                 height);  // deviceMemory, offset, flags and dimensions
    size_t dotPos = m_imageFilename.find_last_of(".");
    string format;
    if (dotPos != string::npos)
        format = m_imageFilename.substr(dotPos + 1);
    else
        format = "unknown";
    std::transform(format.begin(), format.end(), format.begin(), [](unsigned char c) { return std::tolower(c); });
    size_t nPixelBytes = width * height * 4;
    // unsigned char* pixels = new unsigned char[nPixelBytes];
    unsigned char* pixels = (unsigned char*)imageData->dataPointer();
    if (pixels) {
        /*
        size_t k = 0;
        for (size_t i = 0; i < imageData->size(); i++) {
            vsg::ubvec4 pix = imageData->at(i);
            pixels[k++] = pix[0];
            pixels[k++] = pix[1];
            pixels[k++] = pix[2];
            pixels[k++] = pix[3];
        }
         */
        if ((width == m_windowTraits->width) && (height == m_windowTraits->height)) {
            // standard display
            if ((format.compare("png") == 0)) {
                int ans = stbi_write_png(m_imageFilename.c_str(), width, height, 4, pixels, 0);
            } else if ((format.compare("tga") == 0)) {
                int ans = stbi_write_tga(m_imageFilename.c_str(), width, height, 4, pixels);
            } else if ((format.compare("jpg") == 0) || (format.compare("jpeg") == 0)) {
                int ans = stbi_write_jpg(m_imageFilename.c_str(), width, height, 4, pixels, 100);
            } else if ((format.compare("bmp") == 0)) {
                int ans = stbi_write_bmp(m_imageFilename.c_str(), width, height, 4, pixels);
            } else {
                cout << "No screen capture written due to unknown image format. Use png, tga, jpg or bmp!" << endl;
            }

        } else {
            // retina display on the Mac
            size_t nPixelBytesReduced = m_windowTraits->width * m_windowTraits->height * 4;
            unsigned char* pixelsReduced = new unsigned char[nPixelBytesReduced];
            stbir_resize_uint8(pixels, width, height, 0, pixelsReduced, m_windowTraits->width, m_windowTraits->height,
                               0, 4);
            if ((format.compare("png") == 0)) {
                int ans = stbi_write_png(m_imageFilename.c_str(), m_windowTraits->width, m_windowTraits->height, 4,
                                         pixelsReduced, 0);
            } else if ((format.compare("tga") == 0)) {
                int ans = stbi_write_tga(m_imageFilename.c_str(), m_windowTraits->width, m_windowTraits->height, 4,
                                         pixelsReduced);
            } else if ((format.compare("jpg") == 0) || (format.compare("jpeg") == 0)) {
                int ans = stbi_write_jpg(m_imageFilename.c_str(), m_windowTraits->width, m_windowTraits->height, 4,
                                         pixelsReduced, 100);
            } else if ((format.compare("bmp") == 0)) {
                int ans = stbi_write_bmp(m_imageFilename.c_str(), m_windowTraits->width, m_windowTraits->height, 4,
                                         pixelsReduced);
            } else {
                cout << "No creen capture written due to unknown image format. Use png, tga, jpg or bmp!" << endl;
            }
            delete[] pixelsReduced;
        }
        // delete[] pixels;
    }
}

void ChVisualSystemVSG::BindAll() {
    cout << "BindAll() called!" << endl;
    if(!m_system) {
        cout << "No system attached, nothing to bind!" << endl;
        return;
    }
    if(m_system->Get_bodylist().size() < 1) {
        cout << "Attached system must have at least 1 rigid body, nothing to bind!" << endl;
        return;
    }
}

void ChVisualSystemVSG::SetLightDirection(double acimut, double elevation) {
    if(!m_initialized) {
        m_acimut = acimut;
        m_elevation = ChClamp(elevation, 0.0, CH_C_PI_2);
    }else {
        cout << "SetLightDirection() cannot be used after initializing!" << endl;
    }
}

void ChVisualSystemVSG::SetLightIntensity(double intensity) {
    if(!m_initialized) {
        m_light_intensity = ChClamp(intensity,0.0,1.0);
    } else {
        cout << "SetLightIntensity() cannot be used after initializing!" << endl;
    }
}
}  // namespace vsg3d
}  // namespace chrono