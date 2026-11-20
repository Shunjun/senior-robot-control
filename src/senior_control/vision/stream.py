"""//Compatible: same sessions/errors/backend, with one management implementation."""

from vehicle_manager.media.stream import MAX_CANDIDATE_CHARS as MAX_CANDIDATE_CHARS
from vehicle_manager.media.stream import SERVER_CODECS as SERVER_CODECS
from vehicle_manager.media.stream import AiortcPeerFactory as AiortcPeerFactory
from vehicle_manager.media.stream import CameraSource as CameraSource
from vehicle_manager.media.stream import NoCommonCodec as NoCommonCodec
from vehicle_manager.media.stream import PeerConnection as PeerConnection
from vehicle_manager.media.stream import PeerFactory as PeerFactory
from vehicle_manager.media.stream import PeerRequest as PeerRequest
from vehicle_manager.media.stream import PeerStats as PeerStats
from vehicle_manager.media.stream import SessionDescription as SessionDescription
from vehicle_manager.media.stream import VideoBusy as VideoBusy
from vehicle_manager.media.stream import VideoDenied as VideoDenied
from vehicle_manager.media.stream import VideoError as VideoError
from vehicle_manager.media.stream import VideoLimits as VideoLimits
from vehicle_manager.media.stream import VideoSession as VideoSession
from vehicle_manager.media.stream import VideoStatus as VideoStatus
from vehicle_manager.media.stream import VideoUnavailable as VideoUnavailable
from vehicle_manager.media.stream import Viewer as Viewer
from vehicle_manager.media.stream import ViewerBudget as ViewerBudget
from vehicle_manager.media.stream import _AiortcPeer as _AiortcPeer
from vehicle_manager.media.stream import _EncodeLoop as _EncodeLoop
from vehicle_manager.media.stream import _is_relay_candidate as _is_relay_candidate
from vehicle_manager.media.stream import _load_aiortc as _load_aiortc
from vehicle_manager.media.stream import _lower_bitrate_cap as _lower_bitrate_cap
from vehicle_manager.media.stream import _make_track as _make_track
from vehicle_manager.media.stream import _OffLoopPeer as _OffLoopPeer
from vehicle_manager.media.stream import _only_relay_candidates as _only_relay_candidates
from vehicle_manager.media.stream import _to_av_frame as _to_av_frame
from vehicle_manager.media.stream import logger as logger
from vehicle_manager.media.stream import negotiate_codec as negotiate_codec
from vehicle_manager.media.turn import IceCredentials as IceCredentials
from vehicle_manager.media.turn import TurnCredentialIssuer as TurnCredentialIssuer

from senior_control.image_frame import ImageFrame as ImageFrame
