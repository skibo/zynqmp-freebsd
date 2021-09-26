diff --git a/sys/arm64/intel/firmware.c b/sys/arm64/intel/firmware.c
index 083caaf94755..b65990364b80 100644
--- a/sys/arm64/intel/firmware.c
+++ b/sys/arm64/intel/firmware.c
@@ -58,7 +58,8 @@ firmware_probe(device_t dev)
 	 * The firmware node has no property compatible.
 	 * Look for a known child.
 	 */
-	if (!fdt_depth_search_compatible(node, "intel,stratix10-svc", 0))
+	if (!fdt_depth_search_compatible(node, "intel,stratix10-svc", 0) &&
+	    !fdt_depth_search_compatible(node, "xlnx,zynqmp-firmware", 0))
 		return (ENXIO);
 
 	if (!ofw_bus_status_okay(dev))
