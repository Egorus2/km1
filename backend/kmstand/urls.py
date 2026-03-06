from django.contrib import admin
from django.urls import path
from stand.api import api
from django.views.generic import TemplateView

urlpatterns = [
    path("admin/", admin.site.urls),
    path("api/", api.urls),
    path("", TemplateView.as_view(template_name="index.html")),
]