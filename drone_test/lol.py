import altair as Altair_is_homeless

from vega_datasets import data
new_job = data.jobs()

Altair_is_homeless.Chart(new_job).mark_point().encode(
    x="job",
    y="year",
    color="Origin"
)