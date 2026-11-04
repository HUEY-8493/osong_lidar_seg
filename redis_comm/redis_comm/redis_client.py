import json
import logging
import sys
from typing import Any, Dict, List, Optional, Tuple, Union

# Ensure real redis package is imported even if shadowed by local redis.py
if 'redis' in sys.modules and not hasattr(sys.modules['redis'], 'Redis'):
    del sys.modules['redis']

try:
    import redis
    if not hasattr(redis, 'Redis'):
        raise AttributeError("Shadowed redis module")
except (ImportError, AttributeError):
    import importlib.util
    import os
    for p in sys.path:
        if p and not p.rstrip('/').endswith('sjjin'):
            pkg_dir = os.path.join(p, 'redis')
            init_file = os.path.join(pkg_dir, '__init__.py')
            if os.path.isfile(init_file):
                spec = importlib.util.spec_from_file_location("redis", init_file, submodule_search_locations=[pkg_dir])
                if spec and spec.loader:
                    redis = importlib.util.module_from_spec(spec)
                    sys.modules['redis'] = redis
                    spec.loader.exec_module(redis)
                    break

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] [%(levelname)s] %(message)s')
logger = logging.getLogger("RedisCommClient")


def decode_safe(val: Any) -> str:
    """Safely decode bytes or return string without raising UnicodeDecodeError."""
    if isinstance(val, bytes):
        return val.decode("utf-8", errors="replace")
    return str(val)


class RedisCommClient:
    """Wrapper class for communicating with Redis server, handling Streams, Hashes, ZSets, and Lists."""

    def __init__(
        self,
        host: str = "172.16.0.200",
        port: int = 6379,
        password: Optional[str] = "1qaz",
        db: int = 0,
        default_stream: str = "SP-OS-RW-R1",
        socket_timeout: float = 3.0,
    ):
        self.host = host
        self.port = int(port)
        self.password = password if password else None
        self.db = int(db)
        self.default_stream = default_stream
        self.socket_timeout = socket_timeout
        self.client: Optional[redis.Redis] = None

    def connect(self) -> bool:
        """Attempt connection and ping Redis server."""
        try:
            self.client = redis.Redis(
                host=self.host,
                port=self.port,
                password=self.password,
                db=self.db,
                socket_timeout=self.socket_timeout,
                decode_responses=False,  # Binary-safe to handle arbitrary data without decode errors
            )
            self.client.ping()
            logger.info(f"Successfully connected to Redis at {self.host}:{self.port} (DB {self.db})")
            return True
        except redis.AuthenticationError:
            logger.error(f"Redis Authentication failed! Please check password for {self.host}:{self.port}")
            self.client = None
            return False
        except (redis.ConnectionError, redis.TimeoutError) as e:
            logger.error(f"Failed to connect to Redis at {self.host}:{self.port} (Timeout/Unreachable): {e}")
            self.client = None
            return False
        except Exception as e:
            logger.error(f"Unexpected error while connecting to Redis: {e}")
            self.client = None
            return False

    def is_connected(self) -> bool:
        if not self.client:
            return False
        try:
            return bool(self.client.ping())
        except Exception:
            return False

    def switch_db(self, new_db: int) -> bool:
        """Switch current database index."""
        self.db = int(new_db)
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
        return self.connect()

    def get_keyspace_info(self) -> Dict[str, Dict[str, int]]:
        """Get summary of keys in all active databases from INFO keyspace."""
        if not self.is_connected() and not self.connect():
            return {}
        try:
            info = self.client.info("keyspace")
            result = {}
            for k, v in info.items():
                if k.startswith("db"):
                    result[k] = v
            return result
        except Exception as e:
            logger.error(f"Error fetching keyspace info: {e}")
            return {}

    def get_all_keys(self, pattern: str = "*") -> List[str]:
        """Fetch list of all keys currently in Redis matching pattern."""
        if not self.is_connected() and not self.connect():
            return []
        try:
            pattern_bytes = pattern.encode("utf-8")
            raw_keys = self.client.keys(pattern_bytes)
            decoded_keys = [decode_safe(k) for k in raw_keys]
            return sorted(decoded_keys)
        except Exception as e:
            logger.error(f"Error fetching keys with pattern '{pattern}': {e}")
            return []

    def get_key_type(self, key_name: str) -> str:
        """Get the data type of a given key."""
        if not self.is_connected() and not self.connect():
            return "none"
        try:
            k_bytes = key_name.encode("utf-8")
            return decode_safe(self.client.type(k_bytes))
        except Exception as e:
            logger.error(f"Error checking type of key '{key_name}': {e}")
            return "none"

    def get_stream_length(self, stream_name: Optional[str] = None) -> int:
        target_stream = (stream_name or self.default_stream).encode("utf-8")
        if not self.is_connected() and not self.connect():
            return 0
        try:
            return self.client.xlen(target_stream)
        except Exception as e:
            return 0

    def read_stream_list(
        self,
        stream_name: Optional[str] = None,
        count: int = 10,
        reverse: bool = True,
    ) -> List[Dict[str, Any]]:
        target_stream = (stream_name or self.default_stream).encode("utf-8")
        if not self.is_connected() and not self.connect():
            return []

        try:
            if reverse:
                raw_messages = self.client.xrevrange(target_stream, max=b"+", min=b"-", count=count)
            else:
                raw_messages = self.client.xrange(target_stream, min=b"-", max=b"+", count=count)

            parsed_list: List[Dict[str, Any]] = []
            for msg_id, fields in raw_messages:
                decoded_fields = {}
                for fk, fv in fields.items():
                    decoded_fields[decode_safe(fk)] = decode_safe(fv)

                entry = {
                    "id": decode_safe(msg_id),
                    "stream": decode_safe(target_stream),
                    "fields": decoded_fields,
                    "parsed_payload": None,
                }

                if "payload" in decoded_fields:
                    try:
                        entry["parsed_payload"] = json.loads(decoded_fields["payload"])
                    except Exception:
                        entry["parsed_payload"] = decoded_fields["payload"]

                parsed_list.append(entry)

            return parsed_list
        except Exception as e:
            logger.error(f"Error reading stream list: {e}")
            return []

    def read_any_key(self, key_name: str) -> Dict[str, Any]:
        """Read any Redis key type and return formatted representation."""
        if not self.is_connected() and not self.connect():
            return {"type": "none", "data": None}

        k_bytes = key_name.encode("utf-8")
        k_type = decode_safe(self.client.type(k_bytes))

        try:
            if k_type == "stream":
                msgs = self.read_stream_list(key_name, count=20)
                return {"type": "stream", "data": msgs, "length": self.get_stream_length(key_name)}
            elif k_type == "hash":
                raw_h = self.client.hgetall(k_bytes)
                h_dict = {}
                for k, v in raw_h.items():
                    key_str = decode_safe(k)
                    val_str = decode_safe(v)
                    # Attempt json decode
                    try:
                        h_dict[key_str] = json.loads(val_str)
                    except Exception:
                        h_dict[key_str] = val_str
                return {"type": "hash", "data": h_dict, "length": len(h_dict)}
            elif k_type == "zset":
                raw_z = self.client.zrange(k_bytes, 0, -1, withscores=True)
                z_list = []
                for val, score in raw_z:
                    z_list.append({"value": decode_safe(val), "score": score})
                return {"type": "zset", "data": z_list, "length": len(z_list)}
            elif k_type == "list":
                raw_l = self.client.lrange(k_bytes, 0, -1)
                l_list = []
                for item in raw_l:
                    s = decode_safe(item)
                    try:
                        l_list.append(json.loads(s))
                    except Exception:
                        l_list.append(s)
                return {"type": "list", "data": l_list, "length": len(l_list)}
            elif k_type == "set":
                raw_s = self.client.smembers(k_bytes)
                return {"type": "set", "data": [decode_safe(x) for x in raw_s], "length": len(raw_s)}
            elif k_type == "string":
                val = self.client.get(k_bytes)
                val_str = decode_safe(val)
                try:
                    return {"type": "string", "data": json.loads(val_str), "raw": val_str}
                except Exception:
                    return {"type": "string", "data": val_str, "raw": val_str}
            else:
                return {"type": k_type, "data": None}
        except Exception as e:
            logger.error(f"Error reading key '{key_name}': {e}")
            return {"type": k_type, "error": str(e)}

    def send_stream_data(
        self,
        stream_name: Optional[str] = None,
        data: Union[Dict[str, Any], List[Any], str] = None,
    ) -> Optional[str]:
        target_stream = (stream_name or self.default_stream).encode("utf-8")
        if not self.is_connected() and not self.connect():
            return None

        try:
            if isinstance(data, (dict, list)):
                payload_str = json.dumps(data, ensure_ascii=False)
            else:
                payload_str = str(data)

            msg_id = self.client.xadd(target_stream, {b"payload": payload_str.encode("utf-8")})
            return decode_safe(msg_id)
        except Exception as e:
            logger.error(f"Error sending stream data: {e}")
            return None

    def close(self):
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None
