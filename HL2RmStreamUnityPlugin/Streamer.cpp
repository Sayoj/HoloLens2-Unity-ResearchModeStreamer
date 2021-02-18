#include "pch.h"
#include <locale>
#include <codecvt>
#include <string>

#define DBG_ENABLE_INFO_LOGGING 1
#define DBG_ENABLE_ERROR_LOGGING 1
#define DBG_ENABLE_VERBOSE_LOGGING 0

using namespace winrt::Windows::Networking::Sockets;
using namespace winrt::Windows::Storage::Streams;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Foundation::Numerics;

Streamer::Streamer(
    std::wstring portName,
    const GUID& guid,
    const winrt::Windows::Perception::Spatial::SpatialCoordinateSystem& coordSystem)
{
    m_portName = portName;
    m_worldCoordSystem = coordSystem;

    // Get GUID identifying the rigNode to
    // initialize the SpatialLocator
    SetLocator(guid);

    StartServer();
}

winrt::Windows::Foundation::IAsyncAction Streamer::StartServer()
{
    try
    {
        // The ConnectionReceived event is raised when connections are received.
        m_streamSocketListener.ConnectionReceived({ this, &Streamer::OnConnectionReceived });

        // Start listening for incoming TCP connections on the specified port. You can specify any port that's not currently in use.
        // Every protocol typically has a standard port number. For example, HTTP is typically 80, FTP is 20 and 21, etc.
        // For this example, we'll choose an arbitrary port number.
        co_await m_streamSocketListener.BindServiceNameAsync(m_portName);
#if DBG_ENABLE_INFO_LOGGING
        wchar_t msgBuffer[200];
        swprintf_s(msgBuffer, L"Streamer::StartServer: Server is listening at %ls. \n",
            m_portName.c_str());
        OutputDebugStringW(msgBuffer);
#endif // DBG_ENABLE_INFO_LOGGING

    }
    catch (winrt::hresult_error const& ex)
    {
#if DBG_ENABLE_ERROR_LOGGING
        SocketErrorStatus webErrorStatus{ SocketError::GetStatus(ex.to_abi()) };
        winrt::hstring message = webErrorStatus != SocketErrorStatus::Unknown ?
            winrt::to_hstring((int32_t)webErrorStatus) : winrt::to_hstring(ex.to_abi());
        OutputDebugStringW(L"Streamer::StartServer: Failed to open listener with ");
        OutputDebugStringW(message.c_str());
        OutputDebugStringW(L"\n");
#endif
    }
}

void Streamer::OnConnectionReceived(
    StreamSocketListener /* sender */,
    StreamSocketListenerConnectionReceivedEventArgs args)
{
    m_streamSocket = args.Socket();
    m_writer = args.Socket().OutputStream();
    m_writer.UnicodeEncoding(UnicodeEncoding::Utf8);
    m_writer.ByteOrder(ByteOrder::LittleEndian);

    m_writeInProgress = false;
    m_streamingEnabled = true;
#if DBG_ENABLE_INFO_LOGGING
    wchar_t msgBuffer[200];
    swprintf_s(msgBuffer, L"Streamer::OnConnectionReceived: Received connection at %ls. \n",
        m_portName.c_str());
    OutputDebugStringW(msgBuffer);
#endif // DBG_ENABLE_INFO_LOGGING
}

void Streamer::Send(
    IResearchModeSensorFrame* pSensorFrame,
    ResearchModeSensorType pSensorType)

{
#if DBG_ENABLE_INFO_LOGGING
    OutputDebugStringW(L"Streamer::Send: Received frame for sending!\n");
#endif

    if (!m_streamSocket || !m_writer)
    {
#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"Streamer::SendFrame: No connection.\n");
#endif
        return;
    }
    if (!m_streamingEnabled)
    {
#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"Streamer::SendFrame: Streaming disabled.\n");
#endif
        return;
    }

    // grab the frame info
    ResearchModeSensorTimestamp rmTimestamp;
    winrt::check_hresult(pSensorFrame->GetTimeStamp(&rmTimestamp));
    auto prevTimestamp = rmTimestamp.HostTicks;

    auto timestamp = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(prevTimestamp)));
    auto location = m_locator.TryLocateAtTimestamp(timestamp, m_worldCoordSystem);
    if (!location)
    {
#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"Streamer::SendFrame: Can't locate frame.\n");
#endif
        return;
    }
    const float4x4 rig2worldTransform = make_float4x4_from_quaternion(location.Orientation()) * make_float4x4_translation(location.Position());
    auto absoluteTimestamp = m_converter.RelativeTicksToAbsoluteTicks(HundredsOfNanoseconds((long long)prevTimestamp)).count();

    // grab the frame data
    ResearchModeSensorResolution resolution;
    IResearchModeAccelFrame* pSensorAccelFrame = nullptr;
    size_t outBufferCount;

    // invalidation value for AHAT 
    USHORT maxValue = 4090;

    pSensorFrame->GetResolution(&resolution);

    HRESULT hr = pSensorFrame->QueryInterface(IID_PPV_ARGS(&pSensorAccelFrame));
    if (!pSensorAccelFrame)
    {
#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"Streamer::SendFrame: Failed to grab Accel Sensor frame.\n");
#endif
        return;
    }

    if (pSensorAccelFrame)
    {
        DirectX::XMFLOAT3 sample;
        char printString[1000];
        HRESULT hr = S_OK;

        ResearchModeSensorTimestamp timeStamp;
        UINT64 lastSocTickDelta = 0;
        UINT64 glastSocTick = 1;

        pSensorFrame->GetTimeStamp(&timeStamp);

        if (glastSocTick != 0)
        {
            lastSocTickDelta = timeStamp.HostTicks - glastSocTick;
        }
        glastSocTick = timeStamp.HostTicks;

        hr = pSensorAccelFrame->GetCalibratedAccelaration(&sample);
        if (FAILED(hr))
        {
            return;
        }
        sprintf_s(printString, "####Accel: % 3.4f % 3.4f % 3.4f %f %I64d %I64d\n",
            sample.x,
            sample.y,
            sample.z,
            sqrt(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z),
            (lastSocTickDelta * 1000) / timeStamp.HostTicksPerSecond
        );

        wchar_t* printStringAsWString = nullptr;

        int convertResult = MultiByteToWideChar(CP_UTF8, 0, printString, -1, printStringAsWString, 0);

        if (m_writeInProgress)
        {
#if DBG_ENABLE_VERBOSE_LOGGING
            OutputDebugStringW(L"Streamer::SendFrame: Write already in progress.\n");
#endif
            return;
        }

        m_writeInProgress = true;


        try
        {
            // Write header
            m_writer.WriteUInt64(absoluteTimestamp);
            m_writer.WriteString(printStringAsWString);

            WriteMatrix4x4(rig2worldTransform);

#if DBG_ENABLE_VERBOSE_LOGGING
            OutputDebugStringW(L"Streamer::SendFrame: Trying to store writer...\n");
#endif
            m_writer.StoreAsync();
        }
        catch (winrt::hresult_error const& ex)
        {
            SocketErrorStatus webErrorStatus{ SocketError::GetStatus(ex.to_abi()) };
            if (webErrorStatus == SocketErrorStatus::ConnectionResetByPeer)
            {
                // the client disconnected!
                m_writer == nullptr;
                m_streamSocket == nullptr;
                m_writeInProgress = false;
            }
#if DBG_ENABLE_ERROR_LOGGING
            winrt::hstring message = ex.message();
            OutputDebugStringW(L"Streamer::SendFrame: Sending failed with ");
            OutputDebugStringW(message.c_str());
            OutputDebugStringW(L"\n");
#endif // DBG_ENABLE_ERROR_LOGGING
        }

#if DBG_ENABLE_VERBOSE_LOGGING
        OutputDebugStringW(L"Streamer::SendFrame: Frame sent!\n");
#endif
    }
}

void Streamer::StreamingToggle()
{
#if DBG_ENABLE_INFO_LOGGING
    OutputDebugStringW(L"Streamer::StreamingToggle: Received!\n");
#endif
    if (m_streamingEnabled)
    {
        m_streamingEnabled = false;
    }
    else if (!m_streamingEnabled)
    {
        m_streamingEnabled = true;
    }
#if DBG_ENABLE_INFO_LOGGING
    OutputDebugStringW(L"Streamer::StreamingToggle: Done!\n");
#endif
}

void Streamer::WriteMatrix4x4(
    _In_ winrt::Windows::Foundation::Numerics::float4x4 matrix)
{
    m_writer.WriteSingle(matrix.m11);
    m_writer.WriteSingle(matrix.m12);
    m_writer.WriteSingle(matrix.m13);
    m_writer.WriteSingle(matrix.m14);

    m_writer.WriteSingle(matrix.m21);
    m_writer.WriteSingle(matrix.m22);
    m_writer.WriteSingle(matrix.m23);
    m_writer.WriteSingle(matrix.m24);

    m_writer.WriteSingle(matrix.m31);
    m_writer.WriteSingle(matrix.m32);
    m_writer.WriteSingle(matrix.m33);
    m_writer.WriteSingle(matrix.m34);

    m_writer.WriteSingle(matrix.m41);
    m_writer.WriteSingle(matrix.m42);
    m_writer.WriteSingle(matrix.m43);
    m_writer.WriteSingle(matrix.m44);
}

void Streamer::SetLocator(const GUID& guid)
{
    m_locator = Preview::SpatialGraphInteropPreview::CreateLocatorForNode(guid);
}
