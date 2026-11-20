"""//Compatible: explicit aliases of the sole manager task core implementation."""

from vehicle_manager.tasks.schedule import Schedule as Schedule
from vehicle_manager.tasks.schedule import _aware_utc as _aware_utc
from vehicle_manager.tasks.schedule import _canonical as _canonical
from vehicle_manager.tasks.schedule import _compile as _compile
from vehicle_manager.tasks.schedule import _Cron as _Cron
from vehicle_manager.tasks.schedule import _cron_after as _cron_after
from vehicle_manager.tasks.schedule import _Field as _Field
from vehicle_manager.tasks.schedule import _field as _field
from vehicle_manager.tasks.schedule import _impossible_date as _impossible_date
from vehicle_manager.tasks.schedule import _interval_after as _interval_after
from vehicle_manager.tasks.schedule import _month_has_day as _month_has_day
from vehicle_manager.tasks.schedule import _number as _number
from vehicle_manager.tasks.schedule import _wall_utc as _wall_utc
